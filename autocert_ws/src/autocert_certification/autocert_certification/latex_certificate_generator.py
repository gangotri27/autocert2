#!/usr/bin/env python3
"""Generate clean, professional ISO 9283 PDF certificates using LaTeX."""

import os
import subprocess
import tempfile
import shutil
from typing import Dict, Any, Optional


class LatexCertificateGenerator:

    def __init__(self, template_dir: Optional[str] = None):
        if template_dir is None:
            package_dir = os.path.dirname(os.path.abspath(__file__))
            self.template_dir = os.path.join(package_dir, '..', 'templates')
        else:
            self.template_dir = template_dir

        os.makedirs(self.template_dir, exist_ok=True)
        self._ensure_template_exists()

    def _ensure_template_exists(self):
        # Always (re)write the template from the code below, rather than
        # only when missing. Previously this was a one-time "create if
        # absent" check, which meant a template.tex left over on disk
        # from a prior run silently shadowed any changes made here —
        # you'd edit this file, rebuild, and the PDF wouldn't change.
        template_path = os.path.join(self.template_dir, 'certificate_template.tex')
        self._create_template(template_path)

    def _create_template(self, path: str):
        template = r'''\documentclass[a4paper,landscape,10pt]{article}

% ================= PACKAGES ================= %
\usepackage[table]{xcolor} 
\usepackage[margin=0.6in]{geometry}
\usepackage{graphicx}
\usepackage{multicol}
\usepackage{tabularx}
\usepackage{helvet} 
\usepackage{titlesec}
\usepackage{amssymb}
\usepackage{enumitem} 
\usepackage{qrcode}
\usepackage[hidelinks]{hyperref} 

\renewcommand{\familydefault}{\sfdefault}

% ================= COLORS ================= %
\definecolor{maincolor}{HTML}{008080}
\definecolor{rowcolor}{HTML}{E6F2F2}
\definecolor{white}{HTML}{FFFFFF}

% ================= LAYOUT ================= %
\setlength{\columnsep}{0.6in}
\setlength{\parindent}{0pt}
\setlength{\tabcolsep}{6pt}
\renewcommand{\arraystretch}{1.3}
\pagestyle{empty}
\raggedcolumns

% ================= SECTION STYLE ================= %
\newcommand{\sectitle}[1]{
    \vspace{0.4cm}
    \colorbox{rowcolor}{
        \parbox{\linewidth}{
            \textbf{\textcolor{maincolor}{#1}}
        }
    }
    \vspace{0.2cm}
}

\begin{document}

% ================= HEADER ================= %
\noindent
\colorbox{maincolor}{
    \parbox{\dimexpr\linewidth-2\fboxsep}{
        \vspace{8pt}
        
        % Top row
        {\color{white}\Large\textbf{{{ROBOT_NAME}}}} \hfill
        {\color{white}\small Research System Datasheet}
        
        \vspace{2pt}
        
        % Bottom row (Model + Revision)
        {\color{white}\small Model: {{MODEL}}} \hfill
        {\color{white}\small Rev {{SOFTWARE_VERSION}} | {{YEAR}}}
        
        \vspace{8pt}
    }
}
\vspace{0.4cm}

\begin{multicols}{3}

% ========================================== %
% PAGE 1 - COLUMN 1: Image & Pitch
% ========================================== %
\begin{center}
    \includegraphics[width=0.7\linewidth, height=4.5cm, keepaspectratio]{robo_1.png}
\end{center}

\vspace{0.2cm}
{\fontsize{26}{28}\selectfont \textbf{{{ROBOT_NAME}}}} \\

{{SUBTITLE_BLOCK}}

\bigskip
\textcolor{maincolor}{\rule{\linewidth}{1pt}}
\smallskip

{{DESCRIPTION}}

\vfill\null
\columnbreak

% ========================================== %
% PAGE 1 - COLUMN 2: Features & Performance
% ========================================== %
\sectitle{Key Features}
\begin{itemize}[leftmargin=*, nosep]
{{KEY_FEATURES_ITEMS}}
\end{itemize}

{{MODIFICATIONS_SECTION}}

\sectitle{Applications}
\begin{itemize}[leftmargin=*, nosep]
{{APPLICATIONS_ITEMS}}
\end{itemize}

\sectitle{Performance}

\colorbox{rowcolor}{
\parbox{\linewidth}{
\rowcolors{1}{white}{rowcolor}
\begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.45\linewidth} X}
Max TCP speed & {{MAX_TCP_SPEED}} m/s \\
Max joint speed & {{MAX_JOINT_SPEED}}$^\circ$/s \\
Mounting Options & {{MOUNTING_OPTIONS}}\\
Noise Level & {{NOISE_DB}} \\
Orientational Accuracy & $\pm$ {{MEAN_ORIENT_ACCURACY}} $^\circ$\\
Positional Accuracy & $\pm$ {{MEAN_ACCURACY}} mm\\
\end{tabularx}
}
}

\vfill\null
\columnbreak

% ========================================== %
% PAGE 1 - COLUMN 3: General & Joint Specs
% ========================================== %
\sectitle{General Specification}

\colorbox{rowcolor}{
\parbox{\linewidth}{
\rowcolors{1}{white}{rowcolor}
\begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.42\linewidth} X}
Payload & {{PAYLOAD}} kg (at max reach) \\
Reach & {{REACH}} mm\\
Degrees of Freedom & {{DOF}} \\
Configuration & {{CONFIGURATION}} \\
Repeatability & $\pm${{MEAN_REPEATABILITY}} mm \\
Footprint & {{FOOTPRINT}} \\
\end{tabularx}
}
}
\smallskip
{\footnotesize *Payload includes end-effector weight. Reach defined as maximum distance from base to TCP.}

\sectitle{Joint Specifications}

\colorbox{rowcolor}{
\parbox{\linewidth}{
\rowcolors{1}{white}{rowcolor}
\begin{tabularx}{\linewidth}{l X X X}
\textbf{Joint} & \textbf{Working Range} & \textbf{Max Speed} & \textbf{Peak Torque} \\
{{JOINT_TABLE}}
\end{tabularx}
}
}
\smallskip
{\footnotesize *Gripper provides additional actuation DOF. Torque values are derived from manufacturer datasheets.}

% ========================================== %
% ================= PAGE 2 ================= %
% ========================================== %
\vfill\null
\pagebreak

% ========================================== %
% PAGE 2 - COLUMN 1 (Col 4 overall): Control
% ========================================== %
\sectitle{Control System}

\colorbox{rowcolor}{
\parbox{\linewidth}{
\rowcolors{1}{white}{rowcolor}
\begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.38\linewidth} X}
Supported OS & Ubuntu 22.04 / 24.04 \\
Middleware & ROS 2 (Humble / Jazzy) \\
Motion Planning & MoveIt 2 (OMPL) \\
Controller & ros2\_control \\
Simulation & Gazebo (Ignition) \\
Programming & Python, C++ APIs \\
Interface & USB / Serial (via controller) \\
\end{tabularx}
}
}

\sectitle{Actuators}

\colorbox{rowcolor}{
\parbox{\linewidth}{
\rowcolors{1}{white}{rowcolor}
\begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.38\linewidth} X}
Actuator Type & {{ACTUATOR_TYPE}} \\
Motor Models & {{MOTOR_MODELS}} \\
Control Mode & {{CONTROL_MODE}} \\
Communication & {{ACTUATOR_COMMUNICATION}} \\
Feedback & {{FEEDBACK}} \\
\end{tabularx}
}
}

\sectitle{Electrical Specifications}

\colorbox{rowcolor}{
\parbox{\linewidth}{
\rowcolors{1}{white}{rowcolor}
\begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.4\linewidth} X}
Input voltage & {{INPUT_VOLTAGE}} \\
Power supply & {{POWER_SUPPLY}} \\
Typical power & {{POWER_CONSUMPTION}} \\
Maximum power & {{MAX_POWER}} (peak load) \\
Digital I/O & {{DIGITAL_IO}}\\
\end{tabularx}
}
}

\vfill\null
\columnbreak

% ========================================== %
% PAGE 2 - COLUMN 2 (Col 5 overall): Hardware
% ========================================== %

\begin{center}
    \includegraphics[width=0.95\linewidth, height=4.5cm, keepaspectratio]{robo_2.png}
\end{center}
\vspace{-0.2cm}

\sectitle{Physical \& Kinematics}

\colorbox{rowcolor}{
\parbox{\linewidth}{
\rowcolors{1}{white}{rowcolor}
\begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.35\linewidth} X}
Materials & {{MATERIAL}} \\
Cable length & {{CABLE_LENGTH}} \\
Total Weight & $\approx$ {{WEIGHT}} kg \\
IP Rating & {{IP_RATING}} \\
Operating Temp & {{OPERATING_TEMP}} \\
Humidity & {{HUMIDITY}} \\
\end{tabularx}
}
}

\sectitle{Safety \& Compliance}

\colorbox{rowcolor}{
\parbox{\linewidth}{
\rowcolors{1}{white}{rowcolor}
\begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.35\linewidth} X}
Safety rating & {{SAFETY_RATING}} \\
Architecture & Software-limited force/torque feedback (PL-b equivalent) \\
Protection & Configurable joint, velocity, and torque limits \\
E-Stop & Software-triggered (external hardware optional) \\
\end{tabularx}
}
}

\vfill\null
\columnbreak

% ========================================== %
% PAGE 2 - COLUMN 3 (Col 6 overall): Logistics
% ========================================== %

\sectitle{Known Limitations}
\begin{itemize}[leftmargin=*, nosep]
{{KNOWN_LIMITATIONS_ITEMS}}
\end{itemize}

\sectitle{Documentation \& Resources}

\colorbox{rowcolor}{
\parbox{\linewidth}{
\begin{tabularx}{\linewidth}{c X}
\vspace{2pt} \qrcode[height=2.3cm]{{{REPO_URL}}} \vspace{2pt} &
\textbf{GitHub Repository} \newline
Access source code, ROS 2 packages, setup guides, and updates. \newline
{\scriptsize \href{{{REPO_URL}}}{{{REPO_URL_SHORT}}}} \\
\end{tabularx}
}
}

\sectitle{Package Contents}

\begin{itemize}[leftmargin=*, nosep]
{{PACKAGE_CONTENTS_ITEMS}}
\end{itemize}

\sectitle{Contact \& Origin}

\colorbox{rowcolor}{
\parbox{\linewidth}{
\rowcolors{1}{white}{rowcolor}
\begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.25\linewidth} X}
Lab/Entity & \textbf{{{COMPANY}}} \\
Contact & \href{mailto:{{EMAIL}}}{{{EMAIL}}} \\
Location & {{COUNTRY}}\\
\end{tabularx}
}
}

\vfill\null
\end{multicols}

\end{document}
'''
        with open(path, 'w') as f:
            f.write(template)

    @staticmethod
    def _resolve_image(source: Optional[str], dest_path: str) -> bool:
        """Places an image at dest_path from either a local file path or
        a URL (auto-detected), whichever the user gave. Returns True on
        success, False if there was no usable source (caller falls back
        to a placeholder)."""
        if not source:
            return False
        source = source.strip()
        if source.startswith("http://") or source.startswith("https://"):
            try:
                import urllib.request
                req = urllib.request.Request(source, headers={"User-Agent": "Mozilla/5.0"})
                with urllib.request.urlopen(req, timeout=15) as resp, open(dest_path, "wb") as out:
                    shutil.copyfileobj(resp, out)
                return os.path.getsize(dest_path) > 0
            except Exception as e:
                print(f"WARNING: could not download image from '{source}': {e}")
                return False
        expanded = os.path.expanduser(source)
        if os.path.exists(expanded):
            shutil.copy(expanded, dest_path)
            return True
        print(f"WARNING: image path not found: {source}")
        return False

    def generate_pdf(self, data: Dict[str, Any], output_path: str, tex_output_path: Optional[str] = None) -> bool:
        """Generates the PDF certificate, and also saves the underlying
        .tex source next to it (default: same name/folder as
        output_path, with a .tex extension — override with
        tex_output_path if you want it somewhere else).

        The .tex is written out even if pdflatex fails, so a failed
        build can still be inspected/fixed by hand.
        """
        if tex_output_path is None:
            tex_output_path = os.path.splitext(output_path)[0] + ".tex"

        with tempfile.TemporaryDirectory() as tmpdir:
            tex = self._prepare_latex_content(data)
            tex_path = os.path.join(tmpdir, "cert.tex")

            with open(tex_path, "w") as f:
                f.write(tex)

            # Persist the .tex regardless of whether pdflatex succeeds below.
            os.makedirs(os.path.dirname(os.path.abspath(tex_output_path)) or ".", exist_ok=True)
            shutil.copy(tex_path, tex_output_path)

            # Image 1 (Front/Primary) — file path or URL, auto-detected
            img_path_1 = os.path.join(tmpdir, "robo_1.png")
            if not self._resolve_image(data.get("robot_image"), img_path_1):
                from PIL import Image
                Image.new("RGB", (400, 400), "#E6F2F2").save(img_path_1)

            # Image 2 (Secondary/Kinematics) — file path or URL, auto-detected
            img_path_2 = os.path.join(tmpdir, "robo_2.png")
            if not self._resolve_image(data.get("robot_image_2"), img_path_2):
                from PIL import Image
                Image.new("RGB", (400, 400), "#E6F2F2").save(img_path_2)

            result = subprocess.run(["pdflatex", "-interaction=nonstopmode", "cert.tex"],
                cwd=tmpdir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )

            pdf_path = os.path.join(tmpdir, "cert.pdf")
            if os.path.exists(pdf_path):
                shutil.copy(pdf_path, output_path)
                return True
            else:
                # If the PDF wasn't created, print the pdflatex error log!
                print("===== PDFLATEX ERROR LOG =====")
                print(result.stdout)
                print("==============================")
                return False

    @staticmethod
    def _escape_latex(text: str) -> str:
        """Escape the handful of LaTeX-special characters likely to show
        up in free-text user input (&, %, #, _, etc.) so a stray
        character in a description or feature doesn't break the build."""
        if text is None:
            return ""
        replacements = (
            ("\\", r"\textbackslash{}"), ("&", r"\&"), ("%", r"\%"),
            ("$", r"\$"), ("#", r"\#"), ("_", r"\_"), ("{", r"\{"),
            ("}", r"\}"), ("~", r"\textasciitilde{}"), ("^", r"\textasciicircum{}"),
        )
        for old, new in replacements:
            text = text.replace(old, new)
        return text

    def _itemize_block(self, items, escape=True) -> str:
        """Builds the \\item lines for a itemize list. Empty list -> a
        single neutral placeholder line so \\begin{itemize}...\\end{itemize}
        never renders with zero items (which LaTeX errors on)."""
        if not items:
            return r"\item \textit{Not specified}"
        lines = []
        for it in items:
            text = self._escape_latex(str(it)) if escape else str(it)
            lines.append(f"    \\item {text}")
        return "\n".join(lines)

    def _prepare_latex_content(self, data):
        template_path = os.path.join(self.template_dir, 'certificate_template.tex')
        with open(template_path) as f:
            template = f.read()

        joints = data.get("joints", [])
        joint_lines = []
        for i, j in enumerate(joints[:6]):
            name = j.get('name', f'J{i+1}')
            rng = j.get('range', '±180°')
            speed = j.get('speed', '275°/s')
            torque = j.get('torque') or '4.1 Nm'  # last-resort fallback only
            joint_lines.append(f"{name} & {rng} & {speed} & {torque} \\\\")

        subtitle = data.get("subtitle") or ""
        subtitle_block = (
            f"{{\\large \\textcolor{{maincolor}}{{{self._escape_latex(subtitle)}}}}}"
            if subtitle else ""
        )

        description = data.get("description") or (
            f"The {data.get('robot_name', 'robot')} is a {data.get('dof', 6)}-DOF "
            "manipulator platform for research and educational applications."
        )

        repo_url = data.get("repo_url") or "https://github.com"
        repo_url_short = data.get("repo_url_short") or repo_url.replace("https://", "").replace("http://", "")

        modifications = data.get("modifications") or []
        if modifications:
            modifications_section = (
                "\\sectitle{Modifications vs. Original}\n"
                "\\begin{itemize}[leftmargin=*, nosep]\n"
                f"{self._itemize_block(modifications)}\n"
                "\\end{itemize}"
            )
        else:
            modifications_section = ""

        replacements = {
            "{{ROBOT_NAME}}": self._escape_latex(data.get("robot_name", "Robot")),
            "{{MODEL}}": self._escape_latex(data.get("model", "Model-X")),
            "{{SOFTWARE_VERSION}}": data.get("software_version", "1.0"),
            "{{YEAR}}": data.get("year", "2024"),
            "{{SUBTITLE_BLOCK}}": subtitle_block,
            "{{DESCRIPTION}}": self._escape_latex(description),
            "{{DOF}}": data.get("dof", 6),
            "{{KEY_FEATURES_ITEMS}}": self._itemize_block(data.get("key_features")),
            "{{MODIFICATIONS_SECTION}}": modifications_section,
            "{{APPLICATIONS_ITEMS}}": self._itemize_block(data.get("applications")),
            "{{MAX_TCP_SPEED}}": f"{data.get('max_tcp_speed', 0.3):.2f}",
            "{{MAX_JOINT_SPEED}}": data.get("max_joint_speed", 275),
            "{{MOUNTING_OPTIONS}}": self._escape_latex(data.get("mounting_options", "Tabletop or fixed base")),
            "{{NOISE_DB}}": self._escape_latex(data.get("noise_db", "< 60 dB(A)")),
            "{{MEAN_ORIENT_ACCURACY}}": f"{data.get('mean_orientation_accuracy_deg', 0.0):.3f}",
            "{{MEAN_ACCURACY}}": f"{data.get('mean_accuracy_mm', 0.0):.3f}",
            "{{PAYLOAD}}": data.get("payload_kg", 0.3),
            "{{REACH}}": int(data.get("reach_m", 0.485) * 1000),
            "{{CONFIGURATION}}": self._escape_latex(data.get("configuration", "Articulated Manipulator")),
            "{{MEAN_REPEATABILITY}}": f"{data.get('mean_repeatability_mm', 0.0):.3f}",
            "{{FOOTPRINT}}": self._escape_latex(str(data.get("footprint") or "Ø150 mm")),
            "{{JOINT_TABLE}}": "\n".join(joint_lines),
            "{{ACTUATOR_TYPE}}": self._escape_latex(data.get("actuator_type", "DYNAMIXEL Smart Servo")),
            "{{MOTOR_MODELS}}": self._escape_latex(data.get("motor_models", "")),
            "{{CONTROL_MODE}}": self._escape_latex(data.get("control_mode", "Position / Velocity / Current")),
            "{{ACTUATOR_COMMUNICATION}}": self._escape_latex(data.get("actuator_communication", "TTL Multi-Drop Bus")),
            "{{FEEDBACK}}": self._escape_latex(data.get("feedback", "Absolute Contactless Encoder")),
            "{{INPUT_VOLTAGE}}": self._escape_latex(data.get("input_voltage", "12 V DC")),
            "{{POWER_SUPPLY}}": self._escape_latex(data.get("power_supply", "12 V, 5 A adapter")),
            "{{POWER_CONSUMPTION}}": self._escape_latex(data.get("power_consumption", "35 W")),
            "{{MAX_POWER}}": self._escape_latex(data.get("max_power", "120 W")),
            "{{DIGITAL_IO}}": self._escape_latex(data.get("digital_io", "Available via external controller")),
            "{{MATERIAL}}": self._escape_latex(data.get("material", "PLA / Aluminum")),
            "{{CABLE_LENGTH}}": self._escape_latex(data.get("cable_length", "1 m")),
            "{{WEIGHT}}": data.get("weight_kg", 0.63),
            "{{IP_RATING}}": self._escape_latex(data.get("ip_rating", "IP20")),
            "{{OPERATING_TEMP}}": self._escape_latex(data.get("operating_temp", "0°C to 45°C")),
            "{{HUMIDITY}}": self._escape_latex(data.get("humidity_range", "5% to 85% (non-condensing)")),
            "{{SAFETY_RATING}}": self._escape_latex(data.get("safety_rating", "Uncertified (Research use only)")),
            "{{KNOWN_LIMITATIONS_ITEMS}}": self._itemize_block(data.get("known_limitations")),
            "{{REPO_URL}}": repo_url,
            "{{REPO_URL_SHORT}}": repo_url_short,
            "{{PACKAGE_CONTENTS_ITEMS}}": self._itemize_block(data.get("package_contents")),
            "{{COMPANY}}": self._escape_latex(data.get("company", "AutoCert Lab")),
            "{{EMAIL}}": data.get("email", "research@domain.edu"),
            "{{COUNTRY}}": self._escape_latex(data.get("country", "India")),
        }

        for k, v in replacements.items():
            template = template.replace(k, str(v))

        return template