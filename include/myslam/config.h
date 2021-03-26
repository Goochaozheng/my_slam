#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace myslam{

class Config{

public:
    ~Config();

    /**
     * @brief Set which file to read config
     * @return true if file opened
     * **/
    static bool SetParameterFile(const std::string &filename);

    // Access parameter by key
    template<typename T>
    static T Get(const std::string &key){
        return T(Config::config_->file_[key]);
    }

private:
    // Private constructor makes a singleton
    Config() {}

    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

};

}

#endif