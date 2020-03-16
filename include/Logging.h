#include <stdio.h>
#include <string>
#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"
#define DEBUG_MAIN
#define DEBUG_BRAINIAC
#define DEBUG_SERVO

const std::string no_color("39m");
const std::string red("31m");
const std::string green("32m");
const std::string yellow("33m");
const std::string blue("34m");
const std::string magenta("35m");
const std::string cyan("36m");
const std::string reset("\033[0m");
const std::string bold("\033[1;");
const std::string regular("\033[0;");

#ifdef DEBUG_MAIN
//#define LOG_MAIN(f, s) printf(ANSI_COLOR_BLUE "[*MAIN*]:" f ANSI_COLOR_RESET, s);
#define LOG_MAIN(f) std::cout<<bold<<cyan<<"[*MAIN*]:"<<regular<<cyan<<f<<reset<<endl;
#else
#define LOG_MAIN(f, s)
#endif
#ifdef DEBUG_BRAINIAC
#define LOG_BRAINIAC(f) std::cout<<bold<<green<<"[*BOSS*]:"<<regular<<green<<f<<reset<<endl;
#else
#define LOG_BRAINIAC(f, s)
#endif
#ifdef DEBUG_SERVO
#define LOG_SERVO(s,f) std::cout<<bold<<magenta<<"[*SERVO-"<<s<<"*]:"<<regular<<magenta<<f<<reset<<endl;
#else
#define LOG_SERVO(n, f, s)
#endif
