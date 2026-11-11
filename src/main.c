#include "system.h"
#include "buffer.h"
#include "callback.h"
#include "device.h"
#include "thread.h"

static char wait_key = '0';
int g_flag = 1;

void main_core(void){

    device_init();

    buffer_init();

    /**/
    /** server 线程启动前准备停止通知 fd，避免主线程退出时找不到唤醒入口。 */
    if(server_stop_init() != 0){
        fprintf(stderr, "main_core: 初始化 server stop_fd 失败\n");
        exit(-1);
    }
    /**/

    thread_init();

    callback_init();

    printf("main_core 启动，按 q 退出\n");
    while(wait_key != 'q'){
        wait_key = getchar();
    }

    /**/
    /** 先改变循环条件，再通过 eventfd 唤醒阻塞中的 epoll_wait。 */
    g_flag = 0;
    server_stop_notify();
    consumer_stop_notify();
    /**/

    callback_free();
    
    thread_free();

    buffer_free();

    device_free();

    return;
}

int main(void){
    int mainpid, status;
    int retry_count = 0;
    int max_retry_count = 5;

monitor:
    mainpid = fork();

    if(mainpid < 0){
        // 加入错误日志或者打印语句
        perror("fork 失败");
        exit(-1);
    }
    else if(mainpid > 0){
        waitpid(mainpid, &status, 0);
        if(WIFEXITED(status) != 0){
            // 子进程正常退出
            printf("子进程正常执行完毕，父进程退出\n");
            exit(0);
        }
        else{
            printf("检测到子进程异常退出！状态码: %d\n", status);
            retry_count++;

            if(retry_count > max_retry_count){
                fprintf(stderr, "子进程崩溃次数过多，放弃重启，父进程退出\n");
                exit(-1);
            }

            int sleep_time = 1 << (retry_count - 1);
            printf("第 %d 次重启，等待 %d 秒后重试...\n", retry_count, sleep_time);
            sleep(sleep_time);

            goto monitor;
        }
    }
    else{
        main_core();
    }

    return 0;
}
