/*********************************************************************
*
* Software License Agreement (GPLv3)
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*********************************************************************/

#ifndef COMMON_H
#define COMMON_H
#pragma once

#define _USE_MATH_DEFINES

#include <exception>
#include <iostream> // std::cout
#include <sstream> // std::stringstream
#include <math.h>
#include <Eigen/Dense>


#if defined(_WIN32)
#include <Windows.h>

#elif defined(__unix__) || defined(__unix) || defined(unix) ||                 \
        (defined(__APPLE__) && defined(__MACH__))
// #include <X11/Xlib.h>
#include <sys/time.h> /* gethrtime(), gettimeofday() */
#include <time.h>     /* clock_gettime(), time() */
#include <unistd.h>   /* POSIX flags */

#if defined(__MACH__) && defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
#endif

#else
#error "Unable to define common::getRealTime( ) for an unknown OS."
#endif

#define FG_RED "\033[0;31m"
#define FG_GREEN "\033[0;32m"
#define FG_YELLOW "\033[0;33m"
#define FG_BLUE "\033[0;34m"
#define FG_MAGENTA "\033[0;35m"
#define FG_CYAN "\033[0;36m"
#define FG_L_RED "\033[0;91m"
#define FG_L_GREEN "\033[0;92m"
#define FG_L_YELLOW "\033[0;93m"
#define FG_L_BLUE "\033[0;94m"
#define FG_L_MAGENTA "\033[0;95m"
#define FG_L_CYAN "\033[0;96m"

#define FG_B_RED "\033[1;31m"
#define FG_B_GREEN "\033[1;32m"
#define FG_B_YELLOW "\033[1;33m"
#define FG_B_BLUE "\033[1;34m"
#define FG_B_MAGENTA "\033[1;35m"
#define FG_B_CYAN "\033[1;36m"
#define FG_B_L_RED "\033[1;91m"
#define FG_B_L_GREEN "\033[1;92m"
#define FG_B_L_YELLOW "\033[1;93m"
#define FG_B_L_BLUE "\033[1;94m"
#define FG_B_L_MAGENTA "\033[1;95m"
#define FG_B_L_CYAN "\033[1;96m"

#define BG_RED "\033[41m"
#define BG_GREEN "\033[42m"
#define BG_YELLOW "\033[43m"
#define BG_BLUE "\033[44m"
#define BG_MAGENTA "\033[45m"
#define BG_CYAN "\033[46m"
#define BG_L_RED "\033[101m"
#define BG_L_GREEN "\033[102m"
#define BG_L_YELLOW "\033[103m"
#define BG_L_BLUE "\033[104m"
#define BG_L_MAGENTA "\033[105m"
#define BG_L_CYAN "\033[106m"

#define RESET "\033[0m"

namespace common {
// class MyException: public std::exception
// {
//   virtual const char* what() const throw()
//   {
//     return "My exception happened";
//   }
// } myex;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class MyException : public std::exception {
private:
        std::string _msg;

public:
        MyException(std::string msg) : std::exception(), _msg(msg){
        };
        virtual ~MyException() throw(){
        };
        virtual const char *what() const throw() {
                return _msg.c_str();
        }
};

/*!
 * \brief
 * Returns the Fast Inverse Square Root.
 */
template <typename R> R finvsqrt(R number) {
        float num = static_cast<float>(number);
        long i;
        float x2, y;
        const float threehalfs = 1.5F;

        x2 = num * 0.5F;
        y = num;
        i = *(long *)&y;     // evil floating point bit level hacking
        i = 0x5f375a86 - (i >> 1); // Chris Lomont's constant
        y = *(float *)&i;
        y = y * (threehalfs - (x2 * y * y)); // 1st iteration
        //      y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can
        //      be removed

        return static_cast<R>(y);
};

/*
 * Author:  David Robert Nadeau
 * Site:    http://NadeauSoftware.com/
 * License: Creative Commons Attribution 3.0 Unported License
 *          http://creativecommons.org/licenses/by/3.0/deed.en_US
 */

/*!
 * \brief
 * Returns the real time, in microseconds, or -1.0 if an error occurred.
 *
 * Time is measured since an arbitrary and OS-dependent start time.
 * The returned real time is only useful for computing an elapsed time
 * between two calls to this function.
 */
template <typename R> R getRealTime() {
#if defined(_WIN32)

        LARGE_INTEGER tm;
        LARGE_INTEGER freq;

        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&tm);

        tm.QuadPart *= 1000000;
        // tm.QuadPart /= freq.QuadPart;
        R tim = static_cast<R>(tm.QuadPart) / static_cast<R>(freq.QuadPart);

        return static_cast<R>(tim);

// FILETIME tm;
// ULONGLONG t;

// #if defined(NTDDI_WIN8) && NTDDI_VERSION >= NTDDI_WIN8
/* Windows 8, Windows Server 2012 and later. ---------------- */
// GetSystemTimePreciseAsFileTime( &tm );
// #else
/* Windows 2000 and later. ---------------------------------- */
// GetSystemTimeAsFileTime( &tm );
// #endif

// t = ((ULONGLONG)tm.dwHighDateTime << 32) | (ULONGLONG)tm.dwLowDateTime;
// return static_cast<R>((double)t / 10000000.0);

#elif (defined(__hpux) || defined(hpux)) ||                                    \
        ((defined(__sun__) || defined(__sun) || defined(sun)) &&                   \
        (defined(__SVR4) || defined(__svr4__)))
        /* HP-UX, Solaris. ------------------------------------------ */
        return static_cast<R>((double)gethrtime() / 1000.0);

#elif defined(__MACH__) && defined(__APPLE__)
        /* OSX. ----------------------------------------------------- */
        static double timeConvert = 0.0;
        if (timeConvert == 0.0) {
                mach_timebase_info_data_t timeBase;
                (void)mach_timebase_info(&timeBase);
                timeConvert = (double)timeBase.numer / (double)timeBase.denom / 1000.0;
        }
        return static_cast<R>((double)mach_absolute_time() * timeConvert);

#elif defined(_POSIX_VERSION)
/* POSIX. --------------------------------------------------- */
#if defined(_POSIX_TIMERS) && (_POSIX_TIMERS > 0)
        {
                struct timespec ts;
#if defined(CLOCK_MONOTONIC_PRECISE)
                /* BSD. --------------------------------------------- */
                const clockid_t id = CLOCK_MONOTONIC_PRECISE;
#elif defined(CLOCK_MONOTONIC_RAW)
                /* Linux. ------------------------------------------- */
                const clockid_t id = CLOCK_MONOTONIC_RAW;
#elif defined(CLOCK_HIGHRES)
                /* Solaris. ----------------------------------------- */
                const clockid_t id = CLOCK_HIGHRES;
#elif defined(CLOCK_MONOTONIC)
                /* AIX, BSD, Linux, POSIX, Solaris. ----------------- */
                const clockid_t id = CLOCK_MONOTONIC;
#elif defined(CLOCK_REALTIME)
                /* AIX, BSD, HP-UX, Linux, POSIX. ------------------- */
                const clockid_t id = CLOCK_REALTIME;
#else
                const clockid_t id = (clockid_t)-1; /* Unknown. */
#endif /* CLOCK_* */
                if (id != (clockid_t)-1 && clock_gettime(id, &ts) != -1)
                        return static_cast<R>((double)ts.tv_sec + (double)ts.tv_nsec / 1000.0);
                /* Fall thru. */
        }
#endif /* _POSIX_TIMERS */

        /* AIX, BSD, Cygwin, HP-UX, Linux, OSX, POSIX, Solaris. ----- */
        struct timeval tm;
        gettimeofday(&tm, NULL);
        return static_cast<R>((double)tm.tv_sec*1e6 + (double)tm.tv_usec);
#else
        return static_cast<R>(-1.0); /* Failed. */
#endif
};

/* Map angles (:math:`\\theta \in R`) to signed angles
 * (:math:`\\theta \in [-pi, +pi)`).
 */
template <typename Derived>
typename Derived::Scalar wrapToPi(const Eigen::MatrixBase<Derived>& matrix1d)
{
        // typename Derived::Scalar angle = matrix1d(0,0);
        if (matrix1d.size() != 1)
        {
                std::stringstream ss;
                ss << "Argument dimensions (" << matrix1d.rows() << ", " << matrix1d.cols() << "not supported.";
                throw(common::MyException(ss.str()));
        }
        typename Derived::Scalar angle = matrix1d(0,0);
        while (angle < -M_PI)
                angle += 2 * M_PI;
        while (angle >= M_PI)
                angle -= 2 * M_PI;
        return angle;
        // return (Eigen::MatrixBase<Derived>() << angle).finished();
};

template <typename R>
R wrapScalarToPi(R angle)
{
        while (angle < -M_PI)
                angle += 2 * M_PI;
        while (angle >= M_PI)
                angle -= 2 * M_PI;
        return angle;
};

// double wrapToPi(double angle)
// {
//         while (angle < -M_PI)
//                 angle += 2 * M_PI;
//         while (angle >= M_PI)
//                 angle -= 2 * M_PI;
//         return angle;
// };

// template <typename Derived>
// typename Eigen::Block<Derived, 1, 1> wrapToPi(const Eigen::MatrixBase<Derived>& matrix1d) {
//         typename Derived::Scalar angle = matrix1d(0,0);
//         while (angle < -M_PI)
//                 angle += 2 * M_PI;
//         while (angle >= M_PI)
//                 angle -= 2 * M_PI;
//         return (Eigen::Block<Derived, 1, 1>() << angle).finished();
// };

/* Map angles (:math:`\\theta \in R`) to unsigned angles
 * (:math:`\\theta \in [0, 2\pi)`).
 */
template <typename R> R wrapTo2Pi(R angle)
{
        while (angle < 0.0)
                angle += 2 * M_PI;
        while (angle >= 2 * M_PI)
                angle -= 2 * M_PI;
        return angle;
};

// Unused SFINAE example
// template <typename T>
// class has_size {
// private:
//  typedef char Yes;
//  typedef Yes No[2];

//  template <typename U, U> struct really_has;

//  template<typename C> static Yes& Test(really_has <size_t (C::*)() const,
//  &C::size>*);
//  template<typename C> static Yes& Test(really_has <size_t (C::*)(),
//  &C::size>*);

//  template<typename> static No& Test(...);

// public:
//     static bool const value = sizeof(Test<T>(0)) == sizeof(Yes);
// };

namespace has_insertion_operator_impl {
typedef char no;
typedef char yes[2];

struct any_t {
        template <typename T> any_t(T const &);
};

no operator<<(std::ostream const &, any_t const &);

yes &test(std::ostream &);
no test(no);

template <typename T> struct has_insertion_operator {
        static std::ostream &s;
        static T const &t;
        static bool const value = sizeof(test(s << t)) == sizeof(yes);
};
}

template <typename T>
struct has_insertion_operator
        : has_insertion_operator_impl::has_insertion_operator<T> {};

// A container is not printable but the return of the call has to be
template <typename T, typename A, template <typename, typename> class C>
char safe_print(C<T, A>) {
        return '\0';
};

template <typename E, typename T, typename A,
          template <typename, typename, typename> class S>
std::string safe_print(S<E, T, A> toPrint) {
        return toPrint;
};

template <typename> short safe_print(short toPrint) {
        return toPrint;
};

template <typename> unsigned short safe_print(unsigned short toPrint) {
        return toPrint;
};

template <typename> int safe_print(int toPrint) {
        return toPrint;
};

template <typename> unsigned safe_print(unsigned toPrint) {
        return toPrint;
};

template <typename> long safe_print(long toPrint) {
        return toPrint;
};

template <typename> unsigned long safe_print(unsigned long toPrint) {
        return toPrint;
};

template <typename> float safe_print(float toPrint) {
        return toPrint;
};

template <typename> double safe_print(double toPrint) {
        return toPrint;
};

template <typename> long double safe_print(long double toPrint) {
        return toPrint;
};

template <typename> bool safe_print(bool toPrint) {
        return toPrint;
};

// For all we know, T could be a container thus the safe_print overloads
template <typename T, typename A, template <typename, typename> class C>
void printContainer(C<T, A> toIterate) {
        std::cout << "[";
        for (typename C<T, A>::iterator it = toIterate.begin(); it != toIterate.end();
             ++it) {
                if (*it != toIterate.back())
                        std::cout << safe_print(*it) << ", ";
                else
                        std::cout << safe_print(*it);
        }
        std::cout << "]";
};

// dummy template - string is not iterable
// template <typename E, typename A>
// void printContainer(std::_String_val<E, A> toIterate){};

template <typename T, typename A, template <typename, typename> class C>
void printNestedContainerHelper(C<T, A> toPrint) {
        for (typename C<T, A>::iterator it = toPrint.begin(); it != toPrint.end();
             ++it)
                printNestedContainer(*it);
};

// dummy template - string is not iterable
// template <typename E, typename A>
// void printNestedContainerHelper(std::_String_val<E, A> toPrint){};

template <typename T, template <typename, typename> class C>
void printNestedContainer(C<T, std::allocator<T> > toPrint) {
        // Although we know that with this test will only call printContainer for a
        // container of printable stuff we'll need code it in a safe way so the
        // compiler doesn't find an error if it tries to deduce it for a non iterable
        // container or a container of not printable things, let's say, a _String_val
        // container which is not iterable or a container of container which is not
        // printable
        if (has_insertion_operator<T>::value) {
                printContainer(toPrint);
        }
        // Kind of the same situation here, but only with the not iterable container
        // case
        else {
                std::cout << "[";
                printNestedContainerHelper(toPrint);
                std::cout << "]";
        }
};

template <class M, class V> void ValsOnMapToVec(const M &m, V &v) {
        for (typename M::const_iterator it = m.begin(); it != m.end(); ++it) {
                v.insert(v.end(), it->second);
        }
};

template <class M, class V> void KeysOnMapToVec(const M &m, V &v) {
        for (typename M::const_iterator it = m.begin(); it != m.end(); ++it) {
                v.insert(v.end(), it->first);
        }
};

template <class T, const size_t siz> class CArray
{

private:
        T array[siz];

public:
        const size_t size() const
        {
                return siz;
        };

        template <typename R> T &operator[](R pos)
        {
                if (pos >= siz) {
                        std::stringstream ss;
                        ss << "common::CArray: invalid index. ";
                        throw MyException(ss.str());
                }
                return array[pos];
};
};

typedef CArray<double, 3> CArray3d;
typedef CArray<double, 2> CArray2d;
}

#endif // COMMON_H
