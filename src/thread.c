#include "thread.h"

ThreadInfo thread_map[THREAD_NUM] = {
    {.func = server, .arg = NULL, .valid = false},
    // 生产者作为回调函数，不再额外手动开线程
    // {0, NULL, producer, NULL, false},
    {.func = consumer, .arg = NULL, .valid = false}
};

static const char* thread_func_name(ThreadFunc func){
    if(func == server) return "server";
    if(func == consumer) return "consumer";
    return "unknown";
}

static void print_thread_map(void){
    printf("thread_map:\n");
    printf("+-------+----------------------+----------+----------+-------+-----------------------------------------------+\n");
    printf("| index | id                   | func     | arg      | valid | attr                                          |\n");
    printf("+-------+----------------------+----------+----------+-------+-----------------------------------------------+\n");

    for(int i = 0; i < THREAD_NUM; i++){
        ThreadInfo* thread_info = &thread_map[i];
        int detach_state = -1, scope = -1, inherit_sched = -1, sched_policy = -1;
        size_t stack_size = 0, guard_size = 0;
        void* stack_addr = NULL;
        struct sched_param sched = {0};

        pthread_attr_getdetachstate(&thread_info->attr, &detach_state);
        pthread_attr_getscope(&thread_info->attr, &scope);
        pthread_attr_getinheritsched(&thread_info->attr, &inherit_sched);
        pthread_attr_getschedpolicy(&thread_info->attr, &sched_policy);
        pthread_attr_getschedparam(&thread_info->attr, &sched);
        pthread_attr_getstack(&thread_info->attr, &stack_addr, &stack_size);
        pthread_attr_getguardsize(&thread_info->attr, &guard_size);

        printf("| %-5d | %-20lu | %-8s | %-8p | %-5s | detach=%d scope=%d inherit=%d policy=%d priority=%d stack=%p/%zu guard=%zu |\n",
               i, (unsigned long)thread_info->id,
               thread_func_name(thread_info->func), thread_info->arg,
               thread_info->valid ? "true" : "false",
               detach_state, scope, inherit_sched, sched_policy,
               sched.sched_priority, stack_addr, stack_size, guard_size);
    }

    printf("+-------+----------------------+----------+----------+-------+-----------------------------------------------+\n");
}

void thread_init(void){
    for(int i = 0; i < THREAD_NUM; i++){
        ThreadInfo* thread_info = &thread_map[i];
        if(pthread_attr_init(&thread_info->attr) != 0){
            fprintf(stderr, "thread_init: 初始化线程属性失败\n");
            exit(-1);
        }
        if(pthread_create(&thread_info->id, &thread_info->attr, thread_info->func, thread_info->arg) != 0){
            fprintf(stderr, "thread_init: 创建线程失败\n");
            exit(-1);
        }

        thread_info->valid = true;
    }

    /* 打印 thread_map 二维表 */
    print_thread_map();

    return;
}

void thread_free(void){
    for(int i = 0; i < THREAD_NUM; i++){
        ThreadInfo* thread_info = &thread_map[i];
        if(!thread_info->valid) continue;

        printf("thread_free: 线程 %s 退出\n", thread_func_name(thread_info->func));
        pthread_join(thread_info->id, NULL);
        pthread_attr_destroy(&thread_info->attr);
        thread_info->valid = false;

    }

    return;
}
