#include <iostream>

// 1. 引入 Ruckig
#include <ruckig/ruckig.hpp>

// 2. 引入 Pinocchio
#include <pinocchio/multibody/model.hpp>

// 3. 引入 Lely CANopen (修正了头文件)
#include <lely/ev/loop.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/posix/poll.hpp>

int main() {
    std::cout << "========== 🚀 机器人底层点火测试 ==========\n";

    // ---------------------------------------------------------
    // 1. 测试 Ruckig
    // ---------------------------------------------------------
    ruckig::Ruckig<1> otg(0.001); 
    std::cout << "[OK] Ruckig 轨迹生成器就绪！\n";

    // ---------------------------------------------------------
    // 2. 测试 Pinocchio
    // ---------------------------------------------------------
    pinocchio::Model model;
    std::cout << "[OK] Pinocchio 动力学引擎就绪！\n";

    // ---------------------------------------------------------
    // 3. 测试 Lely CANopen (标准的初始化流程)
    // ---------------------------------------------------------
    lely::io::IoGuard io_guard;             // 初始化 IO 守护
    lely::io::Context ctx;                  // 创建 IO 上下文
    lely::io::Poll poll(ctx);               // 创建轮询器 (必须的)
    lely::ev::Loop loop(poll.get_poll());   // 真正正确的事件循环！
    
    std::cout << "[OK] Lely CANopen 通信栈就绪！\n";

    std::cout << "===========================================\n";
    std::cout << "🎉 完美！赶紧去把 ros_canopen 删了吧！\n";
    
    return 0;
}
