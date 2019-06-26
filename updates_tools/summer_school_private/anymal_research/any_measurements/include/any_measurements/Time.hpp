/*!
 * @file    Time.hpp
 * @author  Philipp Leemann
 * @date    Sep 19, 2016
 * @version 0.0
 *
 */
#pragma once

#include <time.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <cmath> // for modf

namespace any_measurements {

struct Time
{
 public:

    Time():
        time_{0, 0}
    {
    }

    Time(const int64_t seconds, const int64_t nanoSeconds):
        time_{seconds, nanoSeconds}
    {
    }

    /*!
     * Constructor using seconds floating point value
     * @param seconds   Seconds to construct time from
     */
    explicit Time(const double seconds) : Time()
    {
        fromSeconds(seconds);
    }

    /*!
     * Constructor using linux timespec structure
     * @param other     reference to the object to be copied
     */
    explicit Time(const timespec& other):
        time_(other)
    {
    }

    /*!
     * Constructor using std::chrono time_point
     * @param other     reference to the object to be copied
     */
    template <typename ClockType>
    explicit Time(const std::chrono::time_point<ClockType>& other):
        time_()
    {
        *this = other;
    }

    virtual ~Time() = default;

    /*!
     * Assignment operator using std::chrono time_point
     * @param other     reference to the object to be copied
     * @return          reference to this object
     */
    template <typename ClockType>
    inline Time& operator=(const std::chrono::time_point<ClockType>& other) {
        const auto duration = other.time_since_epoch();
        const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
        time_.tv_sec = seconds.count();
        time_.tv_nsec = nanoseconds.count();
        return *this;
    }

    /*!
     * Assignment operator using timespec
     * @param other     reference to the object to be copied
     * @return          reference to this object
     */
    inline Time& operator=(const timespec& other) {
        time_ = other;
        return *this;
    }

    /*!
     * Substraction operator
     * @param other     reference to the object to be substracted
     * @return          time difference
     */
    Time operator-(const Time& other) const {
        int64_t sec = seconds() - other.seconds();
        int64_t nsec = nanoSeconds() - other.nanoSeconds();
        if (nsec < 0) {
            sec--;
            nsec += static_cast<int64_t>(1e9);
        }
        return Time(sec, nsec);
    }

    /*!
     * Sum operator
     * @param other     reference to the Time to be added
     * @return          time sum
     */
    Time operator+(const Time& other) const {
        uint64_t sec = seconds() + other.seconds();
        uint64_t nsec = nanoSeconds() + other.nanoSeconds();
        if (nsec > static_cast<uint64_t>(1e9)) {
            sec++;
            nsec -= static_cast<uint64_t>(1e9);
        }
        return Time(sec, nsec);
    }

    /*!
     * Sum operator
     * @param seconds   time to be added
     * @return          time sum
     */
    Time operator+(const double seconds) const {
        double secD;
        uint64_t nsec = nanoSeconds() + static_cast<uint64_t>(std::modf(seconds, &secD)*1e9);
        uint64_t sec = this->seconds() + static_cast<uint64_t>(secD);
        if (nsec > static_cast<uint64_t>(1e9)) {
            sec++;
            nsec -= static_cast<uint64_t>(1e9);
        }
        return Time(sec, nsec);
    }

    /*!
     * Equal to comparison operator
     * @param other   Time to compare to.
     * @return        True if times are equal.
     */
    bool operator==(const Time& other) const {
        return (time_.tv_sec == other.time_.tv_sec && time_.tv_nsec == other.time_.tv_nsec);
    }

    /*!
     * Less than comparison operator
     * @param other   Time to compare to.
     * @return        True if this time is earlier than other time.
     */
    bool operator<(const Time& other) const {
        return ((time_.tv_sec < other.time_.tv_sec) || (time_.tv_sec == other.time_.tv_sec && time_.tv_nsec < other.time_.tv_nsec));
    }

    /*!
     * Less than or equal to comparison operator
     * @param other   Time to compare to.
     * @return        True if this time is earlier than or equal to the other time.
     */
    bool operator<=(const Time& other) const {
        return *this < other || *this == other;
    }

    /*!
     * Greater than comparison operator
     * @param other   Time to compare to.
     * @return        True if this time is later than other time.
     */
    bool operator>(const Time& other) const {
        return !(*this <= other);
    }

    /*!
     * Greater than or equal to comparison operator
     * @param other   Time to compare to.
     * @return        True if this time is later than or equal to the other time.
     */
    bool operator>=(const Time& other) const {
        return !(*this < other);
    }

    /*!
     * Convert to std::chrono time_point
     * @return      std::chrono::steady_clock::time_point
     */
    inline std::chrono::steady_clock::time_point toChrono() const {
        return std::chrono::steady_clock::time_point(std::chrono::seconds(time_.tv_sec) + std::chrono::nanoseconds(time_.tv_nsec));
    }

    inline void set(const int64_t seconds, const int64_t nanoSeconds) {
        time_.tv_sec = seconds;
        time_.tv_nsec = nanoSeconds;
    }

    inline int64_t seconds() const {
        return time_.tv_sec;
    }

    inline void setSeconds(const int64_t seconds) {
        time_.tv_sec = seconds;
    }

    inline int64_t nanoSeconds() const {
        return time_.tv_nsec;
    }

    inline void setNanoSeconds(const int64_t nanoSeconds) {
        time_.tv_nsec = nanoSeconds;
    }

    inline double toSeconds() const {
        return static_cast<double>(seconds()) + 1e-9*static_cast<double>(nanoSeconds());
    }

    inline void fromSeconds(double seconds) {
        time_.tv_sec = static_cast<int64_t>(seconds);
        time_.tv_nsec = static_cast<int64_t>((seconds - time_.tv_sec) * 1e9);
    }

    inline int64_t toNanoSeconds() const {
        return static_cast<int64_t>(1e9)*seconds() + nanoSeconds();
    }

    inline void fromNanoSeconds(int64_t nanoSeconds) {
        time_.tv_sec = static_cast<int64_t>(1e-9 * nanoSeconds);
        time_.tv_nsec = static_cast<int64_t>(nanoSeconds - time_.tv_sec * 1e9);
    }

    /*!
     * Create a current time point
     * @return          current time point
     */
    static Time Now() {
        Time time;
        clock_gettime(CLOCK_MONOTONIC, &time.getImpl());
        return time;
    }

    /*!
     * Sets the time to the current time using monotonic clock
     * @return          reference to this object
     */
    inline Time& setNow() {
        *this = Now();
        return *this;
    }

    /*!
     * Create a current time point using the wall clock (warning: may jump)
     * @return          current wall clock time point
     */
    static Time NowWallClock() {
        Time time;
        clock_gettime(CLOCK_REALTIME, &time.getImpl());
        return time;
    }

    /*!
     * Sets the time to the current time using the wall clock (warning: may jump)
     * @return          reference to this object
     */
    inline Time& setNowWallClock() {
        *this = NowWallClock();
        return *this;
    }

    /*!
     * Create a current time point using the wall clock (warning: may jump)
     * @return          current wall clock time point
     */
    static Time NowRaw() {
        Time time;
        clock_gettime(CLOCK_MONOTONIC_RAW, &time.getImpl());
        return time;
    }

    /*!
     * Sets the time to the current time using the wall clock (warning: may jump)
     * @return          reference to this object
     */
    inline Time& setNowRaw() {
        *this = NowRaw();
        return *this;
    }

    /*!
     * Create a current time point using the wall clock (warning: may jump)
     * @return          current wall clock time point
     */
    static Time NowCpuProcess() {
        Time time;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time.getImpl());
        return time;
    }

    /*!
     * Sets the time to the current time using the wall clock (warning: may jump)
     * @return          reference to this object
     */
    inline Time& setNowCpuProcess() {
        *this = NowCpuProcess();
        return *this;
    }

    /*!
     * Create a current time point using the wall clock (warning: may jump)
     * @return          current wall clock time point
     */
    static Time NowCpuThread() {
        Time time;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time.getImpl());
        return time;
    }

    /*!
     * Sets the time to the current time using the wall clock (warning: may jump)
     * @return          reference to this object
     */
    inline Time& setNowCpuThread() {
        *this = NowCpuThread();
        return *this;
    }

    /*!
     * @return true if the saved time was obtained using wall clock, false if monotonic clock
     */
    inline bool isWallClock() const {
        return (time_.tv_sec > static_cast<time_t>(1e9)); // if the time was taken using the monotonic clock, the computer would need to run since several thousand days to get to this value...
    }

    /*!
     * Get the elapsed time
     * @return          elapsed time
     */
    Time getElapsedTime() const {
        if (isWallClock()) {
            return (NowWallClock() - *this);
        } else {
            return (Now() - *this);
        }
    }

    Time getElapsedTimeMonotonicRaw() const {
        return (NowRaw() - *this);
    }

    Time getElapsedTimeMonotonic() const {
        return (Now() - *this);
    }

    Time getElapsedTimeWallClock() const {
        return (NowWallClock() - *this);
    }

    Time getElapsedTimeCpuProcess() const {
        return (NowCpuProcess() - *this);
    }

    Time getElapsedTimeCpuThread() const {
        return (NowCpuThread() - *this);
    }

    Time getElapsedTimeAndUpdateMonotonic() {
        Time now = Now();
        Time period = (now - *this);
        *this = now;
        return period;
    }

    timespec& getImpl() {
        return time_;
    }

    const timespec& getImpl() const {
        return time_;
    }

    inline bool isZero() const { return time_.tv_sec == 0 && time_.tv_nsec == 0; }

 protected:
    timespec time_;
};

inline std::ostream& operator<<(std::ostream& os, const Time& time)
{
    return os << std::fixed << std::setprecision(9) << time.toSeconds() << " s";
}


} /* namespace any_measurements */
