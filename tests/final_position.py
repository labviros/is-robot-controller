#!/usr/bin/env python

import argparse
import task


def main():
    parser = argparse.ArgumentParser(description='Final position test.')
    parser.add_argument('-x', type=float, default=0.0)
    parser.add_argument('-y', type=float, default=0.0)
    parser.add_argument('-r', type=float, default=5.0)
    parser.add_argument('-e', type=float, default=0.1)

    options = parser.parse_args()

    final_position = {
        "pose": {"goal": {"position": {"x": options.x, "y": options.y}}}, "allowed_error": options.e,
        "sampling": {"frequency": options.r}
    }

    task.request(final_position)


if __name__ == "__main__":
    main()