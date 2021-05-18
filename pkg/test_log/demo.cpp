#include <glog/logging.h>
int main(int argc, char* argv[])
{
    //Initialize Google's logging library.
    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::INFO,"../");//大于INFO级别的，日志文件放在当前路径的test_log文件夹下面。不设置的话默认在系统/tmp/下面，INFO， ERROR，WARNING按文件分开。
    google::SetStderrLogging(google::INFO);//配置输出到标准错误输出的最低日记级别（输出了所有日志)
    char lv[10] = "l loveyou";
    int num_cookies = 10;
    int addr = 34;
    LOG(INFO) << "Found " << num_cookies <<":--->"<< lv<< " cookies";
    // for(int i = 0; i < 5; ++i)
    // {
    //     LOG(INFO) << "Found " << num_cookies+i<<":--->"<< lv<< " cookies";
    //     LOG(WARNING) <<"warn 地址: "<<&addr;
    //     LOG(ERROR) <<" value: "<<addr;
    // }
    return 0;
}
