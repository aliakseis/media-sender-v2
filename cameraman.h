#pragma once

#include <string>
#include <vector>

struct CameraMode
{
    int w;
    int h;
    int num;
    int den;

    std::string format;

    double fps() const;
    std::string getDescr() const;
};

struct CameraDesc
{
    std::string id;
    std::string description;
    std::string launchLine;
    std::vector<CameraMode> modes;
};

std::vector<CameraDesc> getCameraDescriptions();

