
#include <string.h>

#define __FILE_NAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)


#define DEBUG_HERE std::cerr << __FILE_NAME__ << " : " << __LINE__ << std::endl



#define DEBUG 1

#if DEBUG == 1
#   define DEBUG_DO(x) DEBUG_DO(x)
#   define DEBUG_SHOW(x) do {       std::cerr << __FILE_NAME__ << "." << __LINE__ << ": " << #x << " = " << x << std::endl; } while (0)
#   define DEBUG_PRINTLN(x) do {    std::cerr << __FILE_NAME__ << "." << __LINE__ << ": " << x << std::endl; } while (0)
#else
#   define DEBUG_DO(x)
#   define DEBUG_SHOW(x)
#   define DEBUG_PRINTLN(x)
#endif





#define ENUM(name, ...) enum class name { __VA_ARGS__, __COUNT}; \
inline std::ostream& operator<<(std::ostream& os, name value) { \
std::string enumName = #name; \
std::string str = #__VA_ARGS__; \
int len = str.length(); \
std::vector<std::string> strings; \
std::ostringstream temp; \
for(int i = 0; i < len; i ++) { \
if(isspace(str[i])) continue; \
        else if(str[i] == ',') { \
        strings.push_back(temp.str()); \
        temp.str(std::string());\
        } \
        else temp<< str[i]; \
} \
strings.push_back(temp.str()); \
os << enumName << "::" << strings[static_cast<int>(value)]; \
return os;}


