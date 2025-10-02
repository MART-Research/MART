#include <string>
#include <fstream> // Required for std::ifstream
#include <iostream>
#include <unordered_map>
class ConfigLoader
{
private:
    std::unordered_map<std::string, std::string> params;

public:
    void load_config(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            throw std::runtime_error("Unable to open configuration file: " + filename);
        }

        std::string line;
        while (std::getline(file, line))
        {
            // Ignore comments and empty lines
            if (line.empty() || line[0] == '#')
            {
                continue;
            }

            size_t delimiter_pos = line.find('=');
            if (delimiter_pos != std::string::npos)
            {
                std::string key = line.substr(0, delimiter_pos);
                std::string value = line.substr(delimiter_pos + 1);
                // Stop parsing the value before the '#' character
                size_t comment_pos = value.find('#');
                if (comment_pos != std::string::npos)
                {
                    value = value.substr(0, comment_pos);
                }

                // Trim whitespace
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);

                params[key] = value;
            }
        }
        file.close();
    }

    std::string get_param(const std::string &key, const std::string &default_value = "") const {
        auto it = params.find(key);
        if (it != params.end()) {
            return it->second;
        }
        std::cerr << "Warning: Parameter '" << key << "' not found. Using default value: " << default_value << std::endl;
        return default_value;
    }
};