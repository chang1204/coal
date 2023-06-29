/*
 * @Author: changsaier changsaier@icloud.com
 * @Date: 2022-11-02 22:08:04
 * @LastEditors: changsaier changsaier@icloud.com
 * @LastEditTime: 2022-11-02 22:10:41
 * @FilePath: /coal/src/coal_demo/src/glog_test.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <glog/logging.h>
int main(int argc,char* argv[])
{
    // FLAGS_log_dir="/home/chang/coal/src/coal_demo/glogs";
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold=google::INFO;
    FLAGS_colorlogtostderr=true;
    for(int i = 1; i <= 100;i++)
    {
        LOG_IF(INFO,i==100)<<"LOG_IF(INFO,i==100)  google::COUNTER="<<google::COUNTER<<"  i="<<i;
        LOG_EVERY_N(INFO,10)<<"LOG_EVERY_N(INFO,10)  google::COUNTER="<<google::COUNTER<<"  i="<<i;
        LOG_IF_EVERY_N(WARNING,(i>50),10)<<"LOG_IF_EVERY_N(INFO,(i>50),10)  google::COUNTER="<<google::COUNTER<<"  i="<<i;
        LOG_FIRST_N(ERROR,5)<<"LOG_FIRST_N(INFO,5)  google::COUNTER="<<google::COUNTER<<"  i="<<i;
    }
    google::ShutdownGoogleLogging();
}