#ifndef ERRORHANDLING_H_INCLUDED
#define ERRORHANDLING_H_INCLUDED

// using standard exceptions
#include <exception>
using namespace std;

class myexception: public exception
{
  virtual const char* what() const throw()
  {
    return "My exception happened";
  }
};



#endif // ERRORHANDLING_H_INCLUDED
