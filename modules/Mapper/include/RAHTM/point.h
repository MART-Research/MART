/*
    The point class represents a multi-dimensional coordinate in a topological space
    to manipulate, compare, and encode its coordinates.
    Special Features:
    Automatic coordinate encoding/decoding
    Support for normalization in toroidal topologies
    Space-filling curve operations
*/
#ifndef POINT_H
#define POINT_H

#include <stdlib.h>
#include <vector>
#include <map>
#include <ostream>
#include <sstream>
#include <list>

using namespace std;

class point {
public:
    vector<int> pt;       // Coordinate values of the point, (e.g. pt = {2,5,1} means X=2, Y=5, Z=1 in a 3D space)
    static int n;         // Static dimension count for all points (3 --> 3D)
    static int scale;     // Scaling factor for encoding
    long long code;       // Encoded value of the point

    // Constructors/Destructor
    point(int n_dims) { pt.resize(n_dims, 0); code = -1; }  // Constructor with specified dimensions
    point() { pt.resize(n, 0); code = -1; }                 // Default constructor using static dimension
    point(long long val, point &dims) { pt.resize(n, 0); val2cord(val, dims); reset(); } // Constructor from encoded value
    ~point() {}                                             // Destructor
    
    // Resets the encoded value to -1 (unencoded state)
    void reset() { code = -1; }

    // Converts an encoded value back to coordinate representation using given dimensions
    void val2cord(long long val, point &dims) {
        for(int i = 0; i < n; i++) {
            pt[i] = val % dims[i];
            val /= dims[i];
        }
    }

    // Static method to set the dimension count for all points
    static void set_n(int n_dims) { n = n_dims; }

    // Encodes the point as a single value based on given dimensions
    long get_value(point &dims) {
        long value = 0;
        for(int d = dims.size() - 1; d >= 0; d--) { 
            value = value * dims[d] + pt[d]; 
        }
        return value;
    }

    // Checks if this point is bounded between two other points p1 and p2
    bool is_bounded(point &p1, point &p2) {
        for(int d = 0; d < n; d++) {
            if((pt[d] < p1.pt[d] && pt[d] < p2.pt[d]) ||
                (pt[d] > p1.pt[d] && pt[d] > p2.pt[d]))
                return false;
        }
        return true;
    }

    // Sets all coordinates of the point to the same value c
    void set(int c) {
        if(pt.size() < n) pt.resize(n,c);
        for(int d = 0; d < n; d++) pt[d] = c;
    }

    // Increments the point's coordinates (like a counter) within given dimensions
    // Returns false if increment would go out of bounds
    bool inc(point &dims) {
        int carry = 0;
        vector<int> temp = pt;
        for(int d = 0; d < n && dims[d] > 0; d++) {
            int t = pt[d] + carry + (d == 0);
            pt[d] = t % dims[d];
            carry = t / dims[d];
            if(!carry) return true;
        }
        pt = temp;
        return false;
    }

    // Adds another point's coordinates to this point
    void add(point &p) { for(int d = 0; d < n; d++) pt[d] += p.pt[d]; }

    // Calculates the Manhattan distance (number of hops) to another point
    long get_nhops(point &p) {
        long n_hops = 0;
        for(int i = 0; i < n; i++) {
            long d = p.pt[i] - pt[i];
            n_hops += d > 0 ? d : -d;
        }
        return n_hops;
    }
    
    // Arithmetic operations that modify the point's coordinates
    void mul(int c) { for(int d = 0; d < n; d++) pt[d] *= c; }      // Multiply by scalar
    void mul(point &p) { for(int d = 0; d < n; d++) pt[d] *= p.pt[d]; } // Multiply by another point
    void div(int c) { for(int d = 0; d < n; d++) pt[d] /= c; }      // Divide by scalar
    void div(point &p) { for(int d = 0; d < n; d++) pt[d] /= p.pt[d]; } // Divide by another point
    void mod(int c) { for(int d = 0; d < n; d++) pt[d] %= c; }      // Modulo by scalar
    void normalize(point &dims) { for(int d = 0; d < n; d++) pt[d] = (pt[d] + dims[d]) % dims[d]; } // Normalize within dimensions

    // Rotates the coordinates upward, moving the highest dimension to position 0, used for mapping variants
    void rotate_upper(int mxDim) {
        int rep = mxDim-1;
        int t = pt[rep];
        for(int d = rep; d > 0; d--) pt[d] = pt [d - 1];
        pt[0] = t;
    }

    // Operator overloads
    int& operator[](int d) { return pt[d]; }                     // Access coordinate (mutable)
    const int operator[](int d) const{ return pt[d]; }           // Access coordinate (const)
    inline int size(){ return pt.size(); }                       // Get dimension count

    // Encodes the point's coordinates into a single long long value using the static scale
    void encode() { code = 0; for(int d = pt.size() - 1; d >= 0; d--) code = code*scale + pt[d]; }

    // Gets the encoded value (computes it if not already encoded)
    long long get_code() {
        encode();
        return code;
    }

    // Writes the point's coordinates to an output stream with specified delimiter
    void write_point(ostream &f, char delim = '_') {
        f << pt[pt.size() - 1];
        for(int i = pt.size() - 2; i >= 0; i--) {
            f << delim << pt[i];
        }
    }

    string point_to_string(char delim = '_')
    {
        if (this->pt.empty()) return "";

        std::ostringstream oss;
        // highest–index coordinate first
        oss << this->pt[this->pt.size() - 1];
        for (int i = this->pt.size() - 2; i >= 0; --i)
            oss << delim << this->pt[i];
        return oss.str();
    }

    
    // Special formatting for output based on dimension count
    void write_point_special(ostream &f) {
        switch (n) {
        case 3:
            f << pt[3] << " 0 " << pt[2] << " " << pt[1] << " " << pt[0] << " 0";
            break;
        case 4:
            write_point(f, ' ');;
            f << " 0";
            break;
        case 5:
            write_point(f, ' ');
            break;
        }
    }

    // Gets all adjacent neighbors within given dimensions
    void get_neighbors(map<pair<int, int>, point > &neighbors, point &dims);

    // Comparison operator based on encoded value
    bool operator<(const point &other) const { return code < const_cast<point &>(other).get_code(); }

    // Checks if another point is a neighbor (adjacent in exactly one dimension)
    // Returns the dimension where they differ, or -1 if not neighbors
    int isNeighbor(point &p) {
        int dim = -1;
        for(int d = 0; d < pt.size(); d++) {
            long t = pt[d] - p.pt[d];
            if((t == 1) || (t == -1)) {
                if(dim != -1)
                    return -1;
                dim = d;
            } else if(t > 1 || t < -1) {
                return -1;
            }
        }
        return dim;
    }

    // Equality comparison
    bool operator==(const point &other) const {
        bool ret = true;
        for(int d = 0; d < n; d++) {
            ret = ret && pt[d] == other[d];
        }
        return ret;
    }

    // Inequality comparison
    bool operator!=(const point &other) const { return !(*this == other); }

    // Calculates the product of all coordinates
    long get_prod() {
        long ret = 1;
        for(int d = 0; d < n; d++) { ret *= pt[d]; }
        return ret;
    }
    
    // Calculates a direction bitmask indicating which coordinates are greater than this point
    long get_direction(point &dest) {
        long dir = 0;
        for(int d = n-1; d >= 0; d--) dir = (dir << 1) + dest.pt[d] > pt[d];
        return dir;
    }
};

// Static member initialization
int point::n = 2;
int point::scale = 100;

// Implementation of get_neighbors - finds all adjacent points within given dimensions
void point::get_neighbors(map<pair<int, int>, point > &neighbors, point &dims) {
    for(int i = 0; i < dims.size(); i++) {
        if(pt[i] > 0) {
            point p;
            p.pt = pt;
            p.pt[i]--;
            neighbors[pair<int, int>(i,0)] = p;
        }

        if(pt[i] < (dims[i]-1)) {
            point p;
            p.pt = pt;
            p.pt[i]++;
            neighbors[pair<int, int>(i,1)] = p;
        }
    }
}

#endif