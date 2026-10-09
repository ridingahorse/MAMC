/* APP/test/test.h — 两从站【同步】测试的接口
 *
 * 实现在 test.c（原先整个测试塞在这个头文件里，现拆出来，避免每次被包含都
 * 复制一份静态数据）。本测试只做验证，与产品逻辑无关，随便改。
 *
 * 测试内容：两拍交替、转向相反、循环执行 ——
 *   拍1  速度模式（0x6060 = 3），两轴一起 +60 rpm
 *   拍2  位置模式（0x6060 = 1），两轴一起往回走 1 圈
 * 每一拍都验证「两轴的新目标由同一条 SYNC 一起落盘」这个同步判据。
 * 细节（阶段划分、判据含义、转向为何必须相反）见 test.c 文件头。 */
#ifndef _APP_TEST_H_
#define _APP_TEST_H_

#include <stdint.h>

/**
 * @brief  测试主状态机，在主循环里每周期调用一次，非阻塞
 * @param  nowTick  HAL_GetTick() 的当前值（毫秒）
 * @note   内部按传进来的时间戳推进阶段，不用自己的计时源。
 */
void test_mode_switch(uint32_t nowTick);

#endif /* _APP_TEST_H_ */
