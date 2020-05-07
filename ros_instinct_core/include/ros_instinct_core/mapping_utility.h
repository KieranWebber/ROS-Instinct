#pragma once

/**
 * Representation of an integer range i-j.
 * i is inclusive and j is exclusive.
 */
class IntRange
{
  public:
    IntRange(int start, int end) : start_(start), end_(end)
    {
    }
    int getStart() const
    {
        return start_;
    }
    int getEnd() const
    {
        return end_;
    }
    bool inRange(int value, bool endInclusive = false)
    {
        return value >= start_ && (endInclusive ? value <= end_ : value < end_);
    }

  protected:
    int start_;
    int end_;
};

class MappingUtility
{
  public:
    /**
     * @param key Integer range in format i-j
     * @return A new integer range with start i and end j
     */
    static IntRange parseStringKey(std::string key)
    {
        int start;
        int end;
        sscanf(key.c_str(), "%d-%d", &start, &end);
        return { start, end };
    }

    /**
     * Return a pointer to the first range the given value matches
     * @tparam T Type of the object paired with each range
     * @param map Vector of range, object pairs
     * @param value Value to compare against each range
     * @return First matching pair based on its integer range
     */
    template <typename T>
    static std::pair<IntRange, T>* getRangedValue(const std::vector<std::pair<IntRange, T>>& map, int value)
    {
        // This can be sped up with binary search if the vector is known to be sorted.
        for (auto& range : map)
        {
            if (range.first.inRange(value))
            {
                return &range;
            }
        }
        return nullptr;
    }
};