#ifndef SETTINGS_H
#define SETTINGS_H

#include <vector>
#include <map>

#include "utils/floatpoint.h"

#ifndef VERSION
#define VERSION "DEV"
#endif

#define FIX_HORRIBLE_UNION_ALL_TYPE_A    0x01
#define FIX_HORRIBLE_UNION_ALL_TYPE_B    0x02
#define FIX_HORRIBLE_EXTENSIVE_STITCHING 0x04
#define FIX_HORRIBLE_UNION_ALL_TYPE_C    0x08
#define FIX_HORRIBLE_KEEP_NONE_CLOSED    0x10

/**
 * Type of support mesh.
 * Block is vertically down from overhang.
 * Tree is a tree struction with overhang at its leaves.
 */
enum Support_Pattern
{
    SUPPORT_TYPE_BLOCK = 0,
    SUPPORT_TYPE_TREE = 1
};

#ifndef DEFAULT_CONFIG_PATH
#define DEFAULT_CONFIG_PATH "default.cfg"
#endif

#define CONFIG_MULTILINE_SEPARATOR "\"\"\""




class SettingsBase
{
private:
    std::map<std::string, std::string> settings;
    SettingsBase* parent;
public:
    SettingsBase();
    SettingsBase(SettingsBase* parent);

    void copySettings(SettingsBase& other);

    void setSetting(std::string key, std::string value);
    int getSettingInt(std::string key);
    std::string getSetting(std::string key);
};

#endif//SETTINGS_H
