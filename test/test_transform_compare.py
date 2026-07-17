"""
Position Registration Transform Verification Test

Reads source and target xlsx files, applies a given transformation matrix
to source tree points, compares with target points, and visualizes the result.

Usage:
    python test_transform_compare.py --source JFL4.xlsx --target 样地_4.xlsx \
        --transformationMatrix JFL4_to_样地_4_transformationMatrix.txt --output-dir .
"""
import argparse
import numpy as np
import openpyxl
import matplotlib.pyplot as plt
import matplotlib
from pathlib import Path
from scipy.spatial import cKDTree

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

MATCH_THRESHOLD = 5.0  # meters - max distance for a valid match


def parse_args():
    parser = argparse.ArgumentParser(description="Position Registration Transform Verification Test")
    parser.add_argument("--source", "-s", required=True, help="Source xlsx file (tree points)")
    parser.add_argument("--target", "-t", required=True, help="Target xlsx file (tree points)")
    parser.add_argument("--transformationMatrix", "-m", required=True,
                        help="Transformation matrix file (4x4 tab-separated)")
    parser.add_argument("--output-dir", "-o", default=".", help="Output directory (default: current dir)")
    parser.add_argument("--threshold", type=float, default=MATCH_THRESHOLD,
                        help=f"Match threshold in meters (default: {MATCH_THRESHOLD})")
    return parser.parse_args()


def load_transformation_matrix(filepath):
    """Load 4x4 transformation matrix from tab-separated text file."""
    matrix = np.loadtxt(filepath, delimiter='\t')
    if matrix.shape != (4, 4):
        raise ValueError(f"Expected 4x4 matrix, got {matrix.shape}")
    return matrix


def parse_xlsx_tree_points(filepath):
    """Parse tree points from xlsx file. Returns (ids, Nx2 array of (x, y))."""
    wb = openpyxl.load_workbook(filepath, read_only=True, data_only=True)
    ws = wb.active

    ids = []
    coords = []
    for row in ws.iter_rows(min_row=2, values_only=False):
        cells = [c.value for c in row[:3]]
        if cells[0] is None or cells[1] is None or cells[2] is None:
            continue
        try:
            tree_id = int(cells[0])
            x = float(cells[1])
            y = float(cells[2])
            ids.append(tree_id)
            coords.append([x, y])
        except (ValueError, TypeError):
            continue
    wb.close()

    if not ids:
        raise ValueError(f"No valid tree points found in {filepath}")
    return np.array(ids), np.array(coords)


def apply_transform(points_2d, matrix):
    """Apply 4x4 transformation matrix to Nx2 points. Returns Nx2 transformed points."""
    n = points_2d.shape[0]
    # Pad to homogeneous coordinates (N, 4)
    pts_h = np.hstack([points_2d, np.zeros((n, 1)), np.ones((n, 1))])
    # Apply transformation
    transformed = (matrix @ pts_h.T).T
    return transformed[:, :2]


def find_nearest_matches(source_pts, target_pts, threshold):
    """Find nearest-neighbor matches within threshold. Returns list of (src_idx, tgt_idx, dist)."""
    tree = cKDTree(target_pts)
    dists, indices = tree.query(source_pts)
    matches = []
    for i, (d, j) in enumerate(zip(dists, indices)):
        if d <= threshold:
            matches.append((i, int(j), float(d)))
    return matches


def print_statistics(matches, source_ids, target_ids, transformed, target_pts):
    """Print registration quality statistics."""
    if not matches:
        print("No matches found within threshold!")
        return

    errors = np.array([m[2] for m in matches])
    print(f"\n{'='*60}")
    print(f"Registration Quality Statistics")
    print(f"{'='*60}")
    print(f"  Source trees:      {len(source_ids)}")
    print(f"  Target trees:      {len(target_ids)}")
    print(f"  Matched pairs:     {len(matches)} / {len(source_ids)}")
    print(f"  Match rate:        {len(matches)/len(source_ids)*100:.1f}%")
    print(f"  Mean error:        {errors.mean():.4f} m")
    print(f"  Std error:         {errors.std():.4f} m")
    print(f"  Max error:         {errors.max():.4f} m")
    print(f"  Min error:         {errors.min():.4f} m")
    print(f"  Median error:      {np.median(errors):.4f} m")
    print(f"{'='*60}")

    print(f"\nMatched tree pairs (source_id -> target_id, distance):")
    for src_idx, tgt_idx, dist in sorted(matches, key=lambda x: x[2]):
        print(f"  {source_ids[src_idx]:>4d} -> {target_ids[tgt_idx]:>4d}  {dist:.4f} m")


def visualize(source_ids, source_pts, target_ids, target_pts,
              transformed_pts, matches, matrix, output_path):
    """Create visualization plots."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 14))
    fig.suptitle("Position Registration - Transform Verification", fontsize=16, fontweight='normal', y=0.98)
    fig.subplots_adjust(top=0.92)

    # ── Plot 1: Before transformation (source vs target) ──
    ax = axes[0, 0]
    ax.scatter(source_pts[:, 0], source_pts[:, 1], c='red', s=40, alpha=0.7, label='Source (original)', zorder=5)
    ax.scatter(target_pts[:, 0], target_pts[:, 1], c='blue', s=40, alpha=0.7, label='Target', zorder=5)
    ax.set_title("Before Transformation (Source vs Target)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # ── Plot 2: After transformation (transformed source vs target) ──
    ax = axes[0, 1]
    ax.scatter(transformed_pts[:, 0], transformed_pts[:, 1], c='red', s=40, alpha=0.7,
               label='Source (transformed)', zorder=5)
    ax.scatter(target_pts[:, 0], target_pts[:, 1], c='blue', s=40, alpha=0.7, label='Target', zorder=5)
    # Draw match lines
    for src_idx, tgt_idx, dist in matches:
        ax.plot([transformed_pts[src_idx, 0], target_pts[tgt_idx, 0]],
                [transformed_pts[src_idx, 1], target_pts[tgt_idx, 1]],
                'g-', linewidth=0.8, alpha=0.5)
    ax.set_title(f"After Transformation ({len(matches)} matched pairs)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # ── Plot 3: Zoomed view of matched points ──
    ax = axes[1, 0]
    if matches:
        src_matched = transformed_pts[[m[0] for m in matches]]
        tgt_matched = target_pts[[m[1] for m in matches]]
        offsets = tgt_matched - src_matched
        mean_offset = offsets.mean(axis=0)

        # Show residuals as vectors
        for src_idx, tgt_idx, dist in matches:
            sx, sy = transformed_pts[src_idx]
            tx, ty = target_pts[tgt_idx]
            ax.annotate('', xy=(tx, ty), xytext=(sx, sy),
                       arrowprops=dict(arrowstyle='->', color='green', lw=1.5, alpha=0.6))
            ax.plot(sx, sy, 'r.', markersize=8)
            ax.plot(tx, ty, 'b.', markersize=8)

        ax.set_title(f"Matched Pairs Detail (mean offset: {np.linalg.norm(mean_offset):.3f} m)")
    else:
        ax.set_title("No matches found")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # ── Plot 4: Error distribution ──
    ax = axes[1, 1]
    if matches:
        errors = np.array([m[2] for m in matches])
        ax.hist(errors, bins=min(20, len(matches)), color='steelblue', edgecolor='white', alpha=0.8)
        ax.axvline(errors.mean(), color='red', linestyle='--', linewidth=2, label=f'Mean: {errors.mean():.3f} m')
        ax.axvline(np.median(errors), color='orange', linestyle='--', linewidth=2,
                   label=f'Median: {np.median(errors):.3f} m')
        ax.set_title("Registration Error Distribution")
        ax.set_xlabel("Distance Error (m)")
        ax.set_ylabel("Count")
        ax.legend()
    else:
        ax.set_title("No matches for error distribution")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nVisualization saved to: {output_path}")
    plt.close(fig)


def main():
    args = parse_args()

    source_file = args.source
    target_file = args.target
    matrix_file = args.transformationMatrix
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Position Registration - Transform Verification Test")
    print("=" * 60)
    print(f"  Source:      {source_file}")
    print(f"  Target:      {target_file}")
    print(f"  Matrix file: {matrix_file}")
    print(f"  Output dir:  {output_dir}")
    print(f"  Threshold:   {args.threshold} m")

    # 1. Load transformation matrix
    print(f"\n[1/6] Loading transformation matrix: {matrix_file}")
    transform_matrix = load_transformation_matrix(matrix_file)
    print(f"  Matrix:\n{transform_matrix}")

    # 2. Parse files
    print(f"\n[2/6] Parsing source: {source_file}")
    source_ids, source_pts = parse_xlsx_tree_points(source_file)
    print(f"  Found {len(source_ids)} tree points")

    print(f"\n[3/6] Parsing target: {target_file}")
    target_ids, target_pts = parse_xlsx_tree_points(target_file)
    print(f"  Found {len(target_ids)} tree points")

    # 3. Apply transformation
    print(f"\n[4/6] Applying transformation matrix...")
    transformed_pts = apply_transform(source_pts, transform_matrix)
    print(f"  Transformed {len(source_ids)} points")

    # 4. Find matches
    print(f"\n[5/6] Finding nearest-neighbor matches (threshold={args.threshold}m)...")
    matches = find_nearest_matches(transformed_pts, target_pts, args.threshold)
    print(f"  Found {len(matches)} matches")

    # 5. Statistics
    print_statistics(matches, source_ids, target_ids, transformed_pts, target_pts)

    # 6. Visualize
    print(f"\n[6/6] Generating visualization...")
    src_name = Path(source_file).stem
    tgt_name = Path(target_file).stem
    png_path = output_dir / f"{src_name}_to_{tgt_name}_verification.png"
    csv_path = output_dir / f"{src_name}_to_{tgt_name}_matched_pairs.csv"

    visualize(source_ids, source_pts, target_ids, target_pts,
              transformed_pts, matches, transform_matrix, str(png_path))

    # Save matched pairs to CSV
    with open(csv_path, 'w') as f:
        f.write("source_id,target_id,distance_m,source_x,source_y,target_x,target_y,transformed_x,transformed_y\n")
        for src_idx, tgt_idx, dist in sorted(matches, key=lambda x: x[2]):
            f.write(f"{source_ids[src_idx]},{target_ids[tgt_idx]},{dist:.6f},"
                    f"{source_pts[src_idx,0]:.6f},{source_pts[src_idx,1]:.6f},"
                    f"{target_pts[tgt_idx,0]:.6f},{target_pts[tgt_idx,1]:.6f},"
                    f"{transformed_pts[src_idx,0]:.6f},{transformed_pts[src_idx,1]:.6f}\n")
    print(f"Matched pairs saved to: {csv_path}")


if __name__ == "__main__":
    main()
