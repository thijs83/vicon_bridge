#ifndef VICON_BRIDGE_PRINTER_H
#define VICON_BRIDGE_PRINTER_H

#include <iostream>
#include <chrono>

#define BLACK "\033[30m"                /* Black */
#define RED "\033[31m"                  /* Red */
#define GREEN "\033[32m"                /* Green */
#define YELLOW "\033[33m"               /* Yellow */
#define BLUE "\033[34m"                 /* Blue */
#define MAGENTA "\033[35m"              /* Magenta */
#define CYAN "\033[36m"                 /* Cyan */
#define WHITE "\033[37m"                /* White */
#define RESET_COLOR "\033[0m"           /* Reset */


#define PRINT(msg, colour) __TIME_STAMP(), std::cout << colour << msg << RESET_COLOR << std::endl;
 
#define print_log(COLOR, f_, ...) printf(COLOR), __TIME_STAMP(), printf((f_), ##__VA_ARGS__), printf("\n"), printf(RESET_COLOR)

#define VICON_INFO_STREAM(msg) PRINT(msg, WHITE);
#define VICON_WARN_STREAM(msg) PRINT(msg, YELLOW);
#define VICON_ERROR_STREAM(msg) PRINT(msg, RED);

/**
 * @brief Macro to print info message using printf style \n
 * Example: VICON_INFO("Hello %s", "World");
 * specifiers: %s for string 
 * %d for integer 
 * %f for float
 * %c for character
 * %p for pointer
 * %u for unsigned integer 
 */
#define VICON_INFO(msg, ...) print_log(WHITE, msg, ##__VA_ARGS__);
#define VICON_WARN(msg, ...) print_log(YELLOW, msg, ##__VA_ARGS__);
#define VICON_ERROR(msg, ...) print_log(RED, msg, ##__VA_ARGS__);

#define VICON_ASSERT(Expr, Msg) __VICON_ASSERT(#Expr, Expr, __FILE__, __LINE__, Msg)
#define VICON_HOOK std::cout << __FILE__ << " Line " << __LINE__ << std::endl;
#define VICON_HOOK_MSG(MSG) std::cout << __FILE__ << " Line " << __LINE__ << " [Message: " << MSG << "]" << std::endl;


#ifndef NDEBUG
#define VICON_DEBUG_STREAM(msg) PRINT(msg, CYAN);
#else
#define VICON_DEBUG_STREAM(msg)
#endif




inline void __VICON_ASSERT(const char *expr_str, bool expr, const char *file, int line, const char *msg)
{
    if (!expr)
    {
        std::cerr << RED << "Assert failed:\t" << msg << "\n"
                  << "Returned:\t" << expr_str << "\n"
                  << "Source:\t\t" << file << ", line " << line << RESET_COLOR;
        abort();
    }
}

inline void __TIME_STAMP(){
   // Get current time
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_tm = *std::localtime(&now_time_t);

    // Get current nanoseconds
    auto now_timepoint = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    auto epoch = now_timepoint.time_since_epoch();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count() % 1000000000;

    printf("[%02i:%02d:%02d.%09ld] ", now_tm.tm_hour, now_tm.tm_min, now_tm.tm_sec, nanoseconds);
}


#endif