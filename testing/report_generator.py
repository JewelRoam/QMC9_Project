"""
Visualization Report Generator for Test Results.
Generates HTML reports with:
  - Interactive charts (trajectory plots, speed profiles)
  - Performance comparison tables
  - Video/gif generation from recorded frames
  - Statistical summaries

Usage:
  from testing.report_generator import ReportGenerator
  gen = ReportGenerator()
  gen.add_run(summary1, "Config A")
  gen.add_run(summary2, "Config B")
  gen.generate_html("output/report.html")
"""
import json
import base64
import io
from typing import List, Dict, Optional
from dataclasses import asdict
from datetime import datetime

try:
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("[Report] matplotlib not available, charts will be skipped")


class ReportGenerator:
    """
    Generates comprehensive HTML reports from test results.
    """

    def __init__(self, title: str = "Autonomous Driving Test Report"):
        self.title = title
        self.runs: List[Dict] = []
        self.comparisons: List[Dict] = []

    def add_run(self, summary, config_name: str = "unnamed"):
        """Add a test run result."""
        self.runs.append({
            'name': config_name,
            'summary': asdict(summary) if hasattr(summary, '__dataclass_fields__') else summary,
            'timestamp': datetime.now().isoformat(),
        })

    def add_comparison(self, metric: str, values: Dict[str, float], description: str = ""):
        """Add a custom comparison metric."""
        self.comparisons.append({
            'metric': metric,
            'values': values,
            'description': description,
        })

    def generate_html(self, filepath: str):
        """Generate comprehensive HTML report."""
        html = self._build_html()
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(html)
        print(f"[Report] Generated: {filepath}")

    def _build_html(self) -> str:
        """Build the complete HTML document."""
        sections = [
            self._header(),
            self._summary_section(),
            self._comparison_table(),
            self._detailed_results(),
            self._charts_section(),
            self._footer(),
        ]
        return "\n".join(sections)

    def _header(self) -> str:
        """HTML header with styles."""
        return f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>{self.title}</title>
    <style>
        body {{
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
            background: #f5f5f5;
        }}
        h1, h2, h3 {{
            color: #333;
        }}
        .header {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 30px;
            border-radius: 10px;
            margin-bottom: 20px;
        }}
        .card {{
            background: white;
            border-radius: 8px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin: 15px 0;
        }}
        th, td {{
            padding: 12px;
            text-align: left;
            border-bottom: 1px solid #ddd;
        }}
        th {{
            background: #667eea;
            color: white;
        }}
        tr:hover {{
            background: #f5f5f5;
        }}
        .pass {{
            color: #28a745;
            font-weight: bold;
        }}
        .fail {{
            color: #dc3545;
            font-weight: bold;
        }}
        .metric {{
            display: inline-block;
            background: #e9ecef;
            padding: 8px 16px;
            border-radius: 20px;
            margin: 5px;
            font-size: 14px;
        }}
        .metric-value {{
            font-weight: bold;
            color: #667eea;
        }}
        .chart-container {{
            text-align: center;
            margin: 20px 0;
        }}
        .chart-container img {{
            max-width: 100%;
            border-radius: 8px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        }}
        .badge {{
            display: inline-block;
            padding: 4px 8px;
            border-radius: 4px;
            font-size: 12px;
            font-weight: bold;
        }}
        .badge-success {{ background: #d4edda; color: #155724; }}
        .badge-warning {{ background: #fff3cd; color: #856404; }}
        .badge-danger {{ background: #f8d7da; color: #721c24; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>🚗 {self.title}</h1>
        <p>Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        <p>Total Runs: {len(self.runs)}</p>
    </div>
"""

    def _summary_section(self) -> str:
        """Overall summary statistics."""
        if not self.runs:
            return '<div class="card"><p>No data available</p></div>'

        total_runs = len(self.runs)
        passed = sum(1 for r in self.runs if r['summary'].get('passed', False))
        failed = total_runs - passed
        avg_speed = sum(r['summary'].get('average_speed', 0) for r in self.runs) / total_runs
        total_collisions = sum(r['summary'].get('collision_count', 0) for r in self.runs)

        return f"""
    <div class="card">
        <h2>📊 Overall Summary</h2>
        <div>
            <span class="metric">Total Runs: <span class="metric-value">{total_runs}</span></span>
            <span class="metric">Passed: <span class="metric-value" style="color: #28a745;">{passed}</span></span>
            <span class="metric">Failed: <span class="metric-value" style="color: #dc3545;">{failed}</span></span>
            <span class="metric">Avg Speed: <span class="metric-value">{avg_speed:.1f} km/h</span></span>
            <span class="metric">Total Collisions: <span class="metric-value">{total_collisions}</span></span>
        </div>
    </div>
"""

    def _comparison_table(self) -> str:
        """Side-by-side comparison table."""
        if not self.runs:
            return ''

        rows = []
        for run in self.runs:
            s = run['summary']
            status_class = 'pass' if s.get('passed') else 'fail'
            status_text = '✓ PASS' if s.get('passed') else '✗ FAIL'

            rows.append(f"""
            <tr>
                <td><strong>{run['name']}</strong></td>
                <td><span class="{status_class}">{status_text}</span></td>
                <td>{s.get('collision_count', 0)}</td>
                <td>{s.get('near_misses', 0)}</td>
                <td>{s.get('average_speed', 0):.1f}</td>
                <td>{s.get('min_distance_to_obstacle', 0):.2f}</td>
                <td>{s.get('duration_seconds', 0):.1f}s</td>
            </tr>
""")

        return f"""
    <div class="card">
        <h2>📋 Comparison Table</h2>
        <table>
            <thead>
                <tr>
                    <th>Configuration</th>
                    <th>Status</th>
                    <th>Collisions</th>
                    <th>Near Misses</th>
                    <th>Avg Speed (km/h)</th>
                    <th>Min Distance (m)</th>
                    <th>Duration</th>
                </tr>
            </thead>
            <tbody>
                {''.join(rows)}
            </tbody>
        </table>
    </div>
"""

    def _detailed_results(self) -> str:
        """Detailed breakdown of each run."""
        sections = []

        for i, run in enumerate(self.runs, 1):
            s = run['summary']

            # Failure reasons
            failures = s.get('failure_reasons', [])
            failure_html = ''
            if failures:
                failure_list = ''.join(f'<li>{f}</li>' for f in failures)
                failure_html = f'<div style="color: #dc3545;"><strong>Failure Reasons:</strong><ul>{failure_list}</ul></div>'

            # Collision events
            collisions = s.get('collision_events', [])
            collision_html = '<p>No collisions</p>'
            if collisions:
                coll_rows = ''.join(
                    f'<tr><td>{c.get("timestamp", 0):.1f}s</td><td>{c.get("other_actor_type", "unknown")}</td>'
                    f'<td>{c.get("impact_speed", 0):.1f} km/h</td><td><span class="badge badge-{"danger" if c.get("severity") == "severe" else "warning"}">{c.get("severity", "unknown")}</span></td></tr>'
                    for c in collisions
                )
                collision_html = f"""
                <table>
                    <thead><tr><th>Time</th><th>Actor</th><th>Impact Speed</th><th>Severity</th></tr></thead>
                    <tbody>{coll_rows}</tbody>
                </table>
                """

            sections.append(f"""
    <div class="card">
        <h3>Run {i}: {run['name']}</h3>
        {failure_html}
        <h4>Performance Metrics</h4>
        <div>
            <span class="metric">Distance: <span class="metric-value">{s.get('total_distance_m', 0):.1f} m</span></span>
            <span class="metric">Max Speed: <span class="metric-value">{s.get('max_speed', 0):.1f} km/h</span></span>
            <span class="metric">Lane Deviation: <span class="metric-value">{s.get('avg_lane_deviation', 0):.2f} m</span></span>
            <span class="metric">Emergency Time: <span class="metric-value">{s.get('time_in_emergency', 0):.1f}s</span></span>
        </div>
        <h4>Latency Breakdown</h4>
        <div>
            <span class="metric">Perception: <span class="metric-value">{s.get('avg_perception_latency_ms', 0):.1f} ms</span></span>
            <span class="metric">Planning: <span class="metric-value">{s.get('avg_planning_latency_ms', 0):.1f} ms</span></span>
            <span class="metric">Control: <span class="metric-value">{s.get('avg_control_latency_ms', 0):.1f} ms</span></span>
        </div>
        <h4>Collision Events</h4>
        {collision_html}
    </div>
""")

        return '\n'.join(sections)

    def _charts_section(self) -> str:
        """Generate charts if matplotlib is available."""
        if not HAS_MATPLOTLIB or not self.runs:
            return ''

        charts_html = []

        # Speed comparison chart
        try:
            fig, ax = plt.subplots(figsize=(10, 5))
            names = [r['name'] for r in self.runs]
            speeds = [r['summary'].get('average_speed', 0) for r in self.runs]
            collisions = [r['summary'].get('collision_count', 0) for r in self.runs]

            x = range(len(names))
            width = 0.35

            ax.bar([i - width/2 for i in x], speeds, width, label='Avg Speed (km/h)', color='#667eea')
            ax2 = ax.twinx()
            ax2.bar([i + width/2 for i in x], collisions, width, label='Collisions', color='#dc3545')

            ax.set_xlabel('Configuration')
            ax.set_ylabel('Speed (km/h)', color='#667eea')
            ax2.set_ylabel('Collisions', color='#dc3545')
            ax.set_xticks(x)
            ax.set_xticklabels(names, rotation=45, ha='right')
            ax.legend(loc='upper left')
            ax2.legend(loc='upper right')

            plt.tight_layout()
            chart_data = self._fig_to_base64(fig)
            plt.close()

            charts_html.append(f'''
    <div class="card">
        <h2>📈 Speed vs Safety Comparison</h2>
        <div class="chart-container">
            <img src="data:image/png;base64,{chart_data}" alt="Speed vs Collisions">
        </div>
    </div>
''')
        except Exception as e:
            print(f"[Report] Chart generation error: {e}")

        return '\n'.join(charts_html) if charts_html else ''

    def _fig_to_base64(self, fig) -> str:
        """Convert matplotlib figure to base64 string."""
        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=150, bbox_inches='tight')
        buf.seek(0)
        return base64.b64encode(buf.read()).decode('utf-8')

    def _footer(self) -> str:
        """HTML footer."""
        return """
    <div class="card" style="text-align: center; color: #666;">
        <p>Generated by Autonomous Driving Testing Framework</p>
    </div>
</body>
</html>
"""

    def generate_markdown(self, filepath: str):
        """Generate a simple markdown report."""
        lines = [f"# {self.title}\n", f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"]

        lines.append("## Summary\n")
        lines.append("| Run | Status | Collisions | Avg Speed | Passed |\n")
        lines.append("|-----|--------|-----------|-----------|--------|\n")

        for run in self.runs:
            s = run['summary']
            status = "✅" if s.get('passed') else "❌"
            lines.append(f"| {run['name']} | {status} | {s.get('collision_count', 0)} | "
                        f"{s.get('average_speed', 0):.1f} | {s.get('passed', False)} |\n")

        with open(filepath, 'w', encoding='utf-8') as f:
            f.writelines(lines)
        print(f"[Report] Markdown report: {filepath}")


# ==================== Quick Export Functions ====================

def export_summary_to_json(summary, filepath: str):
    """Quick export of a single summary to JSON."""
    with open(filepath, 'w') as f:
        json.dump(asdict(summary), f, indent=2)
    print(f"[Export] Saved to {filepath}")


def compare_configs(summaries: List, names: List[str]) -> str:
    """Quick text comparison of multiple configurations."""
    lines = ["=" * 70, "Configuration Comparison", "=" * 70]

    for name, summary in zip(names, summaries):
        s = summary if isinstance(summary, dict) else asdict(summary)
        lines.extend([
            f"\n{name}:",
            f"  Collisions: {s.get('collision_count', 0)}",
            f"  Near Misses: {s.get('near_misses', 0)}",
            f"  Avg Speed: {s.get('average_speed', 0):.1f} km/h",
            f"  Min Obstacle Dist: {s.get('min_distance_to_obstacle', 0):.2f} m",
            f"  Passed: {'Yes' if s.get('passed') else 'No'}",
        ])

    return '\n'.join(lines)


if __name__ == '__main__':
    print("This module provides report generation functionality.")
    print("Import and use in your test script:")
    print("  from testing.report_generator import ReportGenerator")
    print("  gen = ReportGenerator()")
    print("  gen.add_run(summary, 'Config A')")
    print("  gen.generate_html('output/report.html')")
