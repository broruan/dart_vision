#include <iostream>
#include <vector>
#include <string>
#include <sstream>

int main()
{
    std::string input = "test.example.for..parameter";
    std::vector<std::string> result;
    std::string temp;
    std::stringstream ss(input); 
    while (std::getline(ss, temp, '.')) {
        result.push_back(temp);
    }
    for(const auto & ans : result) {
        std::cout << ans << "\n";
    }

    return 0;
}