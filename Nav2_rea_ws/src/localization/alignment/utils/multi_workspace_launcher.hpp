#pragma once

#include <iostream>
#include <memory>
#include <array>
#include <vector>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <signal.h>

struct LaunchConfig {
    std::string workspace_path;
    std::string package_name;
    std::string launch_file;
    std::string setup_file; // setup.bash or setup.zsh
};

class MultiWorkspaceLauncher {
public:
    explicit MultiWorkspaceLauncher() {}

    bool startLaunch(const LaunchConfig& config) {
        
        if (!validateWorkspace(config)) {
            std::cerr << "Workspace validation failed" << std::endl;
            return false;
        }
        
        try {
            std::string command = buildLaunchCommand(config);
            return executeTempBashCommand(command);
        } catch (const std::exception& e) {
            std::cerr << "Exception occurred: " << e.what() << std::endl;
            return false;
        }
    }

    bool executeExistedBashCommand(const std::string& command) {
        std::string cmd = "gnome-terminal -t \"GUI\" -- bash -c \"" + command + "; exec bash\"";
        int status = system((cmd).c_str());
        
        if (status != 0) {
            std::cerr << "Command failed with status " << status << std::endl;
            return false;
        }
        
        return true;
    }

private:
    bool validateWorkspace(const LaunchConfig& config) {
        // 检查工作空间路径是否存在
        if (!std::filesystem::exists(config.workspace_path)) {
            std::cerr << "Workspace path does not exist: " << config.workspace_path << std::endl;
            return false;
        }
        
        // 检查setup文件是否存在
        std::string setup_path = config.workspace_path + "/install/" + config.setup_file;
        if (!std::filesystem::exists(setup_path)) {
            std::cerr << "Setup file does not exist: " << setup_path << std::endl;
            return false;
        }
        
        return true;
    }

    std::string buildLaunchCommand(const LaunchConfig& config) {
        
        std::stringstream cmd;

        cmd << "gnome-terminal -t \"GUI\" -- bash -c"
            << "\"cd " << config.workspace_path << ";"
            << "source install/" << config.setup_file << ";"
            << "ros2 launch " << config.package_name << " " << config.launch_file << ";"
            << "exec bash";
        
        return cmd.str();
    }

    bool executeTempBashCommand(const std::string& command) {
        std::string shell = "bash";
        
        // 创建临时脚本文件
        std::string temp_script = "/home/eatwhat/tmp_launch_script_" + std::to_string(std::time(nullptr));
        std::ofstream script_file(temp_script);
        script_file << "#!/bin/" << shell << "\n";
        script_file << command << "\n";
        script_file.close();
        
        // 设置执行权限
        std::string chmod_cmd = "chmod +x " + temp_script;
        system(chmod_cmd.c_str());
        
        // 执行脚本 - use system() instead of popen() to ensure process starts properly
        int status = system((temp_script).c_str());
        
        // 清理临时文件
        std::remove(temp_script.c_str());
        
        if (status != 0) {
            std::cerr << "Command failed with status " << status << std::endl;
            return false;
        }
        
        return true;
    }

};

bool killExistingProcess(const std::string& package_name, const std::string& launch_file) {
    // 1. 找到所有可能的进程ID
    std::string find_cmd = "pgrep -f '" + package_name + "'";
    
    // 执行查找命令并获取输出
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(find_cmd.c_str(), "r"), pclose);
    
    if (!pipe) {
        std::cerr << "查找进程失败" << std::endl;
        return false;
    }
    
    // 读取进程ID
    std::vector<int> pids;
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        int pid = std::stoi(buffer.data());
        pids.push_back(pid);
    }
    
    if (pids.empty()) {
        std::cout << "未找到相关进程" << std::endl;
        return true; // 没有进程需要杀死也算成功
    }
    
    // 2. 杀死每个找到的进程
    bool all_killed = true;
    for (int pid : pids) {
        // 首先尝试正常终止
        if (kill(pid, SIGTERM) != 0) {
            std::cerr << "无法发送SIGTERM到进程 " << pid << ": " << strerror(errno) << std::endl;
            
            // 如果正常终止失败，尝试强制终止
            if (kill(pid, SIGKILL) != 0) {
                std::cerr << "无法发送SIGKILL到进程 " << pid << ": " << strerror(errno) << std::endl;
                all_killed = false;
            }
        }
    }
    
    // 3. 等待一段时间确保进程完全终止
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 4. 验证进程是否已经终止
    for (int pid : pids) {
        if (kill(pid, 0) == 0) {
            std::cerr << "进程 " << pid << " 仍然存在" << std::endl;
            all_killed = false;
        }
    }
    
    return all_killed;
}