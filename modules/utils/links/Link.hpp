#ifndef LINK_H
#define LINK_H

#include <iostream>
#include <vector>

class Link
{
private:
    // Link properties
    int node1, node2; // IDs of the two connected nodes
    double latency;   // Latency of the link in milliseconds
    double bandwidth; // Bandwidth of the link in Gbps
    bool IsVisited;   // Status of the link (true = IsVisited, false = failed)

    // Virtual channel (VC) properties
    std::vector<double> vc_loads;      // Current load on each VC in bits
    double max_load;                   // Maximum load observed on the link
    double max_load_per_vc;            // Maximum load observed on the link per VC
    double historical_max_load;        // Maximum load observed in the whole link at any time
    std::vector<double> max_load_per_vc_historical; // Maximum load observed in the whole link at any time per VC

    // Detailed power model parameters
    double capacitance;     // Capacitance of the link (Farads)
    double leakage_current; // Leakage current of the link (Amperes)
    double voltage;         // Supply voltage (Volts)
    double frequency;       // Operating frequency (Hz)
    // Total power (accumulated)
    double total_power;
    // active or not
    bool isActive;

public:
    // === Constructors ===

    // Default constructor
    Link()
        : node1(-1), node2(-1), latency(0.0), bandwidth(0.0), IsVisited(false), max_load(0.0),
          capacitance(1e-12), leakage_current(1e-6), voltage(1.0), frequency(1e9),
          total_power(0.0)
    {
        isActive = true;
        max_load_per_vc = 0.0;
        historical_max_load = 0.0;
    }

    // Parameterized constructor
    Link(int n1, int n2, double lat = 0.1, double bw = 10.0, bool status = false, int num_vcs = 1,
         double cap = 1e-12, double leak = 1e-6, double volt = 1.0, double freq = 1e9)
        : node1(n1), node2(n2), latency(lat), bandwidth(bw), IsVisited(status), vc_loads(num_vcs, 0.0),
          max_load(0.0), max_load_per_vc(0.0), capacitance(cap), leakage_current(leak), voltage(volt),
          frequency(freq), total_power(0.0),max_load_per_vc_historical(num_vcs, 0.0)
    {
        isActive = true; // a link by default will be activated at creation
        historical_max_load = 0.0;
    }

    // === Getters ===
    int get_node1() const { return node1; }
    int get_node2() const { return node2; }
    double get_latency() const { return latency; }
    double get_bandwidth() const { return bandwidth; }
    bool get_status() const { return IsVisited; }
    void set_status(bool val)
    {
        IsVisited = val;
    }
    int get_num_vcs() const { return vc_loads.size(); }
    double get_max_load() const { return max_load; }
    double get_max_load_per_vc() const { return max_load_per_vc; }

    // === Load Management ===
    // Update the load for a specific VC
    void update_vc_load(int vc, double load)
    {
        if (vc >= 0 && vc < vc_loads.size())
        {
            vc_loads[vc] += load;
            if (vc_loads[vc] > max_load_per_vc)
            {
                max_load_per_vc = vc_loads[vc];
            }
            // Track historical max per VC
            if (vc_loads[vc] > max_load_per_vc_historical[vc])
            {
                max_load_per_vc_historical[vc] = vc_loads[vc];
            }
        }
        update_max_load(); // Update the overall max load
    }

    // Getter for historical max load per VC
    double get_max_load_per_vc_historical(int vc) const
    {
        return max_load_per_vc_historical[vc];
    }

    // Update the maximum load on the link
    void update_max_load()
    {
        double current_total_load = 0.0;
        for (double load : vc_loads)
        {
            current_total_load += load;
        }
        // Only update max_load if current_total_load exceeds previous max_load
        if (current_total_load > max_load)
        {
            max_load = current_total_load;
        }
        // Track historical max as well (if needed separately)
        if (current_total_load > historical_max_load)
        {
            historical_max_load = current_total_load;
        }
    }
    double get_historical_max_load() const
    {
        return historical_max_load;
    }
    // Get the load for a specific VC
    double get_vc_load(int vc) const
    {
        if (vc >= 0 && vc < vc_loads.size())
        {
            return vc_loads[vc];
        }
        return 0.0;
    }

    // Calculate the total load on the link
    double get_total_load() const
    {
        double total_load = 0.0;
        for (double load : vc_loads)
        {
            total_load += load;
        }
        return total_load;
    }
    void reset_Link()
    {
        IsVisited = false;
        for (size_t i = 0; i < vc_loads.size(); ++i)
        {
            vc_loads[i] = 0.0;
            max_load_per_vc_historical[i] = 0.0;
        }
        max_load = 0.0;
        max_load_per_vc = 0.0;
        historical_max_load = 0.0;
        total_power = 0.0;
    }
    // === Total Power Management ===

    // Get the total power (accumulated)
    double get_total_power() const
    {
        return total_power;
    }

    // Increment the total power (accumulate power over multiple flows)
    void increment_total_power(double power)
    {
        total_power += power;
    }

    // === Power Modeling Methods ===

    // Set detailed power parameters
    void set_detailed_power_parameters(double cap, double leak, double volt, double freq)
    {
        capacitance = cap;
        leakage_current = leak;
        voltage = volt;
        frequency = freq;
        total_power = leakage_current * voltage; // Initialize total power with static power
    }

    // Calculate dynamic power (detailed model)
    double calculate_detailed_dynamic_power(double activity_factor) const
    {
        return activity_factor * capacitance * voltage * voltage * frequency;
    }

    // Calculate static power (detailed model)
    double calculate_detailed_static_power() const
    {
        return leakage_current * voltage;
    }

    // Calculate total power (detailed model)
    double calculate_detailed_total_power(double activity_factor)
    {
        double dynamic_power = calculate_detailed_dynamic_power(activity_factor);
        double static_power = calculate_detailed_static_power();
        return dynamic_power + static_power;
    }

    // === Debugging and Information ===

    // Display link information (for debugging)
    void print_info() const
    {
        std::cout << "Link: " << node1 << " <-> " << node2
                  << ", Bandwidth: " << bandwidth << " Gbps" << std::endl;
        for (size_t i = 0; i < vc_loads.size(); ++i)
        {
            std::cout << "  VC " << i << " Load: " << vc_loads[i] << " bits" << std::endl;
        }
        std::cout << "  Max Load: " << max_load << " bits" << std::endl;
        // std::cout << "  Total Power (Accumulated): " << get_total_power() << " W" << std::endl;
    }

    void setActive(bool val)
    {
        isActive = val;
    }
    bool getActive() const
    {
        return isActive;
    }
};

#endif // LINK_H