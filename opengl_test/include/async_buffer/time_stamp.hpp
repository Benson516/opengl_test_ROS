#ifndef TIME_STAMP_H
#define TIME_STAMP_H

// Determine if we are going to preint some debug information to std_out
#define __DEGUG__

/*
This is a class of TIME_STAMP::Time, which is targeted to re-implement
the ros::Time class for removing the dependency on ROS.

Result:
- The TIME_STAMP::Time::now() function is actually
  much more precise than ros::Time::now().

*/

//
#ifdef __DEGUG__
    #include <iostream> //
#endif
#include <chrono>
#include <cmath>  // For floor()

// using std::vector;



// Time, similar to ros::Time
//------------------------------------------------------//
namespace TIME_STAMP{

    const long long const_10_9 = 1000000000;
    const long double const_10_neg9 = 0.000000001;
    //
    struct Time{
        // The Time is: ( sec + nsec*10^-9 )
        long long sec;  // <-- This can be negative
        long long nsec; // <-- This one is greater or equals to zero
        // Constructors
        Time():sec(0),nsec(0)
        {}
        Time(long long sec_in, long long nsec_in):sec(sec_in),nsec(nsec_in)
        {
            _correction();
        }
        Time(long double sec_in){
            long double f_sec = std::floor(sec_in);
            sec = (long long)(f_sec);
            nsec = (long long)( (sec_in - f_sec)*(long double)(const_10_9));
            _correction();
        }
        //
        void _correction(){
            if (nsec < 0){
                long long n_nsec = -nsec;
                sec -= (long long)( n_nsec/const_10_9 );
                //
                sec--;
                nsec = const_10_9 - (n_nsec % const_10_9);
            }else if (nsec > const_10_9){ // nsec >= 0
                sec += (long long)( nsec/const_10_9 );
                nsec %= const_10_9;

            }
        }
        //
        long double toSec(){
            return ( (long double)(sec) + (long double)(nsec)*const_10_neg9 );
        }
        // Usage: Time time_A = Time::now();
        static Time now(){
            using namespace std::chrono;
            auto tp_n = high_resolution_clock::now();
            auto tp_sec = time_point_cast<seconds>(tp_n);
            Time time_B;
            time_B.sec = tp_sec.time_since_epoch().count();
            time_B.nsec = duration_cast<nanoseconds>(tp_n - tp_sec).count();
            return time_B;
        }
        // Usage: time_A.set_now(); --> time_A becomes the current time.
        void set_now(){
            using namespace std::chrono;
            auto tp_n = high_resolution_clock::now();
            auto tp_sec = time_point_cast<seconds>(tp_n);
            sec = tp_sec.time_since_epoch().count();
            nsec = duration_cast<nanoseconds>(tp_n - tp_sec).count();
        }


        // Comparison
        bool is_zero() const {
            return ( (sec == 0) && (nsec == 0) );
        }
        bool is_negative() const {
            return (sec < 0);
        }
        bool equal(const Time &time_B) const {
            return ( (sec == time_B.sec) && (nsec == time_B.nsec) );
        }
        bool greater_than(const Time &time_B) const {
            if (sec != time_B.sec)
                return (sec > time_B.sec);
            else // ==
                return (nsec > time_B.nsec);
        }
        bool greater_or_equal(const Time &time_B) const {
            if (sec != time_B.sec)
                return (sec > time_B.sec);
            else // ==
                return (nsec >= time_B.nsec);
        }
        //

        // Math
        Time add(const Time &time_B) const {
            return Time(sec+time_B.sec, nsec+time_B.nsec);
        }
        Time minus(const Time &time_B) const {
            return Time(sec-time_B.sec, nsec-time_B.nsec);
        }
        void increase(const Time &time_B){
            sec += time_B.sec; nsec += time_B.nsec;
            _correction();
        }
        void decrease(const Time &time_B){
            sec -= time_B.sec; nsec -= time_B.nsec;
            _correction();
        }
        Time abs() const {
            if (sec < 0){
                return Time(-sec, -nsec);
            }
            return *this;
        }
        //




        // Operators
        bool operator ==(Time const& time_B){
            return equal(time_B);
        }
        bool operator !=(Time const& time_B) const {
            return !equal(time_B);
        }
        bool operator >(Time const& time_B) const {
            return greater_than(time_B);
        }
        bool operator >=(Time const& time_B) const {
            return greater_or_equal(time_B);
        }
        bool operator <(Time const& time_B) const {
            return !greater_or_equal(time_B);
        }
        bool operator <=(Time const& time_B) const {
            return !greater_than(time_B);
        }
        Time operator +(Time const& time_B) const {
            return add(time_B);
        }
        Time operator -(Time const& time_B) const {
            return minus(time_B);
        }
        Time& operator +=(const Time & time_B){
            increase(time_B);
            return *this;
        }
        Time& operator -=(const Time & time_B){
            decrease(time_B);
            return *this;
        }

    };
}
//------------------------------------------------------//




#endif // TIME_STAMP_H
