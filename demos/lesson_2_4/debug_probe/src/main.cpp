#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

// Lecture demo notes:
// this file intentionally contains two runtime defects. The defects are related
// to malformed input and heap ownership. Exact locations are not marked on
// purpose.

const int EXPECTED_FIELD_COUNT = 3;
const int MAX_LINE_LENGTH = 128;

struct ProbeSample {
    int seq;
    double battery_v;
    int satellites;
};

int split_line(char line[], char* fields[], int max_fields) {
    int count = 0;
    char* cursor = line;

    while (*cursor != '\0' && count < max_fields) {
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
            *cursor = '\0';
            ++cursor;
        }

        if (*cursor == '\0') {
            break;
        }

        fields[count] = cursor;
        ++count;

        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' &&
               *cursor != '\r') {
            ++cursor;
        }
    }

    return count;
}

int parse_int(const char* text) {
    // This keeps parsing visibly simple for a GDB demo.
    if (text[0] == '+') {
        ++text;
    }

    return static_cast<int>(std::strtol(text, nullptr, 10));
}

double parse_double(const char* text) {
    // This mirrors parse_int so both functions are easy to inspect in GDB.
    if (text[0] == '+') {
        ++text;
    }

    return std::strtod(text, nullptr);
}

ProbeSample parse_sample(char line[]) {
    char* fields[EXPECTED_FIELD_COUNT] = {};
    const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT);
    (void)field_count;

    ProbeSample sample{};
    sample.seq = parse_int(fields[0]);
    sample.battery_v = parse_double(fields[1]);
    sample.satellites = parse_int(fields[2]);
    return sample;
}

char* health_label(const ProbeSample& sample) {
    char* label = new char[16];

    if (sample.battery_v < 21.0) {
        std::strcpy(label, "battery_low");
        return label;
    }

    if (sample.satellites < 4) {
        std::strcpy(label, "gps_weak");
        return label;
    }

    std::strcpy(label, "ready");
    return label;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: debug_probe <input_path>\n";
        return 1;
    }

    std::ifstream input{argv[1]};
    if (!input) {
        std::cerr << "error: failed to open input file: " << argv[1] << '\n';
        return 2;
    }

    char line[MAX_LINE_LENGTH];
    if (!input.getline(line, MAX_LINE_LENGTH)) {
        std::cerr << "error: input file is empty\n";
        return 3;
    }

    const ProbeSample sample = parse_sample(line);
    char* health = health_label(sample);

    std::cout << "seq " << sample.seq << '\n';
    std::cout << "battery_v " << sample.battery_v << '\n';
    std::cout << "satellites " << sample.satellites << '\n';
    std::cout << "health " << health << '\n';

    return 0;
}
