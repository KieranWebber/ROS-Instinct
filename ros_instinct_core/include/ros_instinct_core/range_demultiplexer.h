#pragma once

#include <utility>
#include <vector>
#include <algorithm>
#include "mapping_utility.h"

/**
 * Maps a single value using a set of integer ranges used as keys and arbitary objects for the value.
 * Used for segmenting ID spaces to perform demuxing
 *
 * @tparam T Type of value to be stored against each key range
 */
template <typename T>
class RangeBasedDemultiplexer
{
  public:
    RangeBasedDemultiplexer()
    {
    }

    /**
     * Add a new item to the map using raw range start and end values.
     * Overlaps are not permitted but this should be enforced by the user.
     *
     * @param start Start of the range (inclusice)
     * @param end End of the range (exclusive)
     * @param item Value for the Key/Value pair
     */
    virtual void addItem(int start, int end, const T& item)
    {
        addItem(IntRange(start, end), item);
    }

    /**
     * Add a new item to the map using an integer range as the key.
     * Overlaps are not permitted but this should be enforced by the user.
     *
     * @param range Range to use as the Key
     * @param item Value for the Key/Value pair
     */
    virtual void addItem(const IntRange& range, const T& item)
    {
        rangeMap_.emplace_back(range, item);
        std::sort(rangeMap_.begin(), rangeMap_.end(), compareRange);
    }

    /**
     * Returns nullptr if no matching range is found
     *
     * @param value Value to query against the integer ranges
     * @return First Key/Value pair the value is in the range for
     */
    virtual std::pair<IntRange, T>* getMatching(int value)
    {
        // This could be sped up using binary search instead of just looping through
        // Perfectly fast enough for now though
        for (auto& range : rangeMap_)
        {
            if (range.first.inRange(value))
            {
                return &range;
            }
        }
        return nullptr;
    }

  protected:
    // Compare two ranges for sorting purposes
    static bool compareRange(const std::pair<IntRange, T>& a, const std::pair<IntRange, T>& b)
    {
        return a.first.getStart() > b.first.getStart();
    }

    std::vector<std::pair<IntRange, T>> rangeMap_;
};