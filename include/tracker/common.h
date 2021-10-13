/* raiviol 8/2021
*/

#ifndef FUSION_TRACKING_COMMMON_H
#define FUSION_TRACKING_COMMMON_H

#include<vector>
#include<thread>

namespace fusion
{
class Common
{
public:
    static inline bool val_found_in_id_arr(
        const unsigned int val,
        const unsigned int arr[],
        const unsigned int len)
    {
        bool found = false;

        for(unsigned int i = 0; i < len; i++){
            if(arr[i] == val)
            {
                found = true;
                break;
            }
        }
        return found;
    }

    static inline void join_threads(std::vector<std::thread>& threads)
    {
        for( std::thread& t : threads )
        {
            t.join();
        }
        threads.clear();
    }
};
} // namespace fusion

#endif // FUSION_TRACKING_COMMMON_H
