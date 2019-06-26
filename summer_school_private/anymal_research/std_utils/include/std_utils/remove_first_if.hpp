/*!
 * @file	 remove_first_if.hpp
 * @author   Philipp Leemann
 * @date	 Mar, 2018
 */

#pragma once

#include <algorithm>

namespace std_utils {

template<class ForwardIt, class UnaryPredicate>
ForwardIt remove_first_if(ForwardIt first, ForwardIt last, UnaryPredicate p)
{
    first = std::find_if(first, last, p);
    if (first != last) {
        for (ForwardIt i = first; ++i != last;) {
            *first++ = std::move(*i);
        }
    }
    return first;
}

} // end namespace
