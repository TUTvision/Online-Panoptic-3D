
#include<common.h>

#include<vector>
#include<thread>

namespace fusion
{

static inline bool Common::val_found_in_id_arr(
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

static inline void Common::join_threads(std::vector<std::thread>& threads)
{
    for( std::thread& t : threads )
    {
        t.join();
    }
}

} // namespace fusion
