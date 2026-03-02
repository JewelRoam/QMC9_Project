"""
Apply Best Configuration - After parameter tuning, update config.yaml.

Usage:
    # After running parameter tuning which saves best_apf_config.json
    python testing/apply_best_config.py

    # Or specify a custom config file
    python testing/apply_best_config.py --best-config output/my_tuning.json

Then run the demo to see the improved performance:
    python -m simulation.single_vehicle_demo
"""
import json
import yaml
import argparse
import shutil
from pathlib import Path


def apply_best_config(best_config_path: str = "output/best_apf_config.json",
                      target_config_path: str = "config/config.yaml"):
    """
    Apply the best APF configuration found during tuning to the main config file.
    """
    # Load best parameters
    if not Path(best_config_path).exists():
        print(f"Error: {best_config_path} not found!")
        print("Run parameter tuning first:")
        print("  python -m testing.automated_runner --tune")
        return False

    with open(best_config_path, 'r') as f:
        best_params = json.load(f)

    print("Loading best configuration from:", best_config_path)
    print("Parameters to apply:")
    for key, value in best_params.items():
        print(f"  {key}: {value}")

    # Load current config
    with open(target_config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    # Backup original config
    backup_path = target_config_path + ".backup"
    shutil.copy(target_config_path, backup_path)
    print(f"\nBackup created: {backup_path}")

    # Update planning.apf section
    if 'planning' not in config:
        config['planning'] = {}
    if 'apf' not in config['planning']:
        config['planning']['apf'] = {}

    # Apply parameters
    apf_config = config['planning']['apf']
    for key, value in best_params.items():
        old_value = apf_config.get(key, "N/A")
        apf_config[key] = value
        print(f"  {key}: {old_value} -> {value}")

    # Save updated config
    with open(target_config_path, 'w', encoding='utf-8') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True)

    print(f"\n✅ Configuration updated: {target_config_path}")
    print("\nNext steps:")
    print("  1. Run single vehicle demo to verify:")
    print("     python -m simulation.single_vehicle_demo")
    print("  2. Or run multi-vehicle demo:")
    print("     python -m simulation.multi_vehicle_demo")

    return True


def compare_configs(config_a_path: str, config_b_path: str):
    """
    Compare two configuration files side by side.
    """
    with open(config_a_path, 'r') as f:
        config_a = yaml.safe_load(f)
    with open(config_b_path, 'r') as f:
        config_b = yaml.safe_load(f)

    apf_a = config_a.get('planning', {}).get('apf', {})
    apf_b = config_b.get('planning', {}).get('apf', {})

    print("\n" + "="*60)
    print("Configuration Comparison (APF Parameters)")
    print("="*60)
    print(f"{'Parameter':<20} {'Config A':>15} {'Config B':>15} {'Diff':>10}")
    print("-"*60)

    all_keys = set(apf_a.keys()) | set(apf_b.keys())
    for key in sorted(all_keys):
        val_a = apf_a.get(key, "N/A")
        val_b = apf_b.get(key, "N/A")

        if isinstance(val_a, (int, float)) and isinstance(val_b, (int, float)):
            diff = f"{((val_b - val_a) / val_a * 100):+.1f}%" if val_a != 0 else "N/A"
        else:
            diff = "-"

        marker = "  "
        if val_a != val_b:
            marker = "* "

        print(f"{marker}{key:<18} {str(val_a):>15} {str(val_b):>15} {diff:>10}")

    print("\n* indicates different values")


def main():
    parser = argparse.ArgumentParser(
        description="Apply best configuration after parameter tuning"
    )
    parser.add_argument(
        "--best-config",
        default="output/best_apf_config.json",
        help="Path to best config JSON from tuning"
    )
    parser.add_argument(
        "--target-config",
        default="config/config.yaml",
        help="Path to target config.yaml to update"
    )
    parser.add_argument(
        "--compare",
        nargs=2,
        metavar=('CONFIG_A', 'CONFIG_B'),
        help="Compare two config files instead of applying"
    )
    parser.add_argument(
        "--restore-backup",
        action="store_true",
        help="Restore config from backup"
    )

    args = parser.parse_args()

    if args.compare:
        compare_configs(args.compare[0], args.compare[1])
    elif args.restore_backup:
        backup_path = args.target_config + ".backup"
        if Path(backup_path).exists():
            shutil.copy(backup_path, args.target_config)
            print(f"Restored {args.target_config} from backup")
        else:
            print(f"No backup found at {backup_path}")
    else:
        apply_best_config(args.best_config, args.target_config)


if __name__ == '__main__':
    main()
