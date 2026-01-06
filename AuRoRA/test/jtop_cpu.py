from jtop import jtop
import pprint

def explore_jetson_keys():
    with jtop() as jetson:
        if not jetson.ok():
            print("Jetson not ready")
            return

        # 1. Print top-level keys
        top_keys = list(jetson.stats.keys())
        print("Top-level keys:", top_keys)

        # 2. For each key, print nested keys if it's a dict
        for key in top_keys:
            value = jetson.stats[key]
            if isinstance(value, dict):
                nested_keys = list(value.keys())
                print(f"\nKey '{key}' has nested keys:", nested_keys)
            else:
                print(f"\nKey '{key}' is not a dict, value type: {type(value)}")

        # 3. Optionally pretty print all data
        print("\nFull jetson.stats:")
        pprint.pprint(jetson.stats)

if __name__ == "__main__":
    explore_jetson_keys()
