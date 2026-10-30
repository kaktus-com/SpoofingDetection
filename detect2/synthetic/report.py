"""Build a portable HTML scientific report with a scrubbable 3D motion view."""

import base64
from html import escape
import json
from pathlib import Path

import numpy as np

from ..geometry import ecef_to_geodetic
from .motions import SCENARIOS


def write_report(output, motions, frame, global_figures, scenario_figures, has_plots):
    def image(path):
        encoded = base64.b64encode((output/path).read_bytes()).decode("ascii")
        return f'<img loading="lazy" src="data:image/png;base64,{encoded}" alt="{escape(path)}">'

    data, sections = {}, []
    for m in motions:
        # Only the display is decimated. CSVs retain every generated sample.
        idx = np.unique(np.linspace(0, len(m.time_s)-1, min(601, len(m.time_s)), dtype=int))
        ecef = frame.enu_to_ecef(m.antenna_position_m)
        llh = ecef_to_geodetic(ecef)
        arrays = {"t": m.time_s, "body": m.body_position_m, "antenna": m.antenna_position_m,
                  "rotation": m.body_to_enu, "rpy": np.degrees(m.rpy_rad), "q": m.quaternion_wxyz,
                  "boresight": m.antenna_boresight_enu, "forward": m.body_forward_enu,
                  "velocity": m.antenna_velocity_m_s, "acceleration": m.antenna_acceleration_m_s2,
                  "specificForce": m.specific_force_body_m_s2, "ecef": ecef,
                  "llh": np.column_stack((np.degrees(llh[:, :2]), llh[:, 2]))}
        name = m.config.scenario
        data[name] = {key: np.round(value[idx], 9).tolist() for key, value in arrays.items()}
        data[name].update(title=SCENARIOS[name], rate=m.config.rate_hz,
                          lever=list(m.config.lever_arm_body_m),
                          orbitFrequencyHz=m.parameters["frequency_hz"] if name == "circle_coning" else None)
        pictures = "".join(image(path) for path in scenario_figures[name])
        sections.append(f'<section class="scenario" data-scenario="{name}"><h2>{escape(name)}</h2>'
                        f'<p><a href="{name}/truth.csv">Truth CSV</a> · '
                        f'<a href="{name}/geometric_projections.csv">Geometric projections CSV</a> · '
                        f'<a href="{name}/config.json">Configuration and metrics</a></p>{pictures}</section>')
    template = (Path(__file__).with_name("report_template.html")).read_text()
    names = [m.config.scenario for m in motions]
    preferred = next((name for name in ["circle_coning", "three_axis"] if name in names), names[0])
    options = "".join(f'<option value="{m.config.scenario}" '
                      f'{"selected" if m.config.scenario == preferred else ""}>'
                      f'{escape(m.config.scenario)}</option>' for m in motions)
    replacements = {"__DATA__": json.dumps(data, allow_nan=False),
                    "__OPTIONS__": options, "__GLOBAL_FIGURES__": "".join(image(p) for p in global_figures),
                    "__SCENARIO_FIGURES__": "".join(sections),
                    "__SITE__": escape(f"{frame.latitude_deg}° latitude, {frame.longitude_deg}° longitude, "
                                       f"{frame.height_m} m ellipsoidal height"),
                    "__PDF__": '<a href="motion_geometry.pdf">Download all figures as PDF</a> · ' if has_plots else ""}
    for marker, value in replacements.items():
        template = template.replace(marker, value)
    (output/"report.html").write_text(template)
