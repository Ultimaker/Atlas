#ifndef CSV_H
#define CSV_H



#include "MACROS.h" // debug

#include "Kernel.h"

#include <initializer_list> // {..}

#include <vector>

#include <ostream> // basic_ostream

// enable/disable debug output
#define CSV_DEBUG 0

#if CSV_DEBUG == 1
#   define CSV_DEBUG_DO(x) DEBUG_DO(x)
#   define CSV_DEBUG_SHOW(x) DEBUG_SHOW(x)
#   define CSV_DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
#else
#   define CSV_DEBUG_DO(x)
#   define CSV_DEBUG_SHOW(x)
#   define CSV_DEBUG_PRINTLN(x)
#endif


//! class used for csv output
template<typename T>
struct CSV
{
    std::vector<std::vector<T>> lines;

    void addLine(std::initializer_list<T> args)
    {
        lines.emplace_back(args);
        //lines.back().insert(args);
    }
    void addLine(std::vector<T>& args)
    {
        lines.push_back(args);
    }

    template<class CharT, class TraitsT>
    friend
    std::basic_ostream<CharT, TraitsT>&
    operator <<(std::basic_ostream<CharT, TraitsT>& os, const CSV& b)
    {
        for (std::vector<T> line : b.lines)
        {
            if (line.size() > 0)
                os<< line[0];
            for (int i = 1; i < line.size(); i++)
                os << ", " << line[i];
            os << std::endl;
        }

        return os;
    };
};

typedef CSV<spaceType> CSVi;

#endif // CSV_H
