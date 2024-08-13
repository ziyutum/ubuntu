```
#include <chrono>
using namespace std::chrono;

auto start = high_resolution_clock::now();
// franka code
auto end = high_resolution_clock::now();
auto duration = duration_cast<microseconds>(end - start).count();
std::cout << "Execution Time: " << duration << " microseconds" << std::endl;
```
