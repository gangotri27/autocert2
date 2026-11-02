"""PDF report generation for certification"""

import os
from datetime import datetime
from typing import Optional

try:
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib.units import inch
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle
    from reportlab.lib.enums import TA_CENTER
    PDF_AVAILABLE = True
except ImportError:
    PDF_AVAILABLE = False

from .certificate import Certificate


class PDFGenerator:
    """Generate PDF certification reports"""
    
    def __init__(self):
        if not PDF_AVAILABLE:
            raise ImportError("reportlab not installed")
            
    def generate(self, certificate: Certificate, output_path: str):
        """Generate PDF report"""
        
        doc = SimpleDocTemplate(output_path, pagesize=A4)
        styles = getSampleStyleSheet()
        story = []
        
        # Custom styles
        title_style = ParagraphStyle(
            'CustomTitle',
            parent=styles['Heading1'],
            fontSize=24,
            spaceAfter=30,
            alignment=TA_CENTER
        )
        
        heading_style = ParagraphStyle(
            'CustomHeading',
            parent=styles['Heading2'],
            fontSize=16,
            spaceAfter=12,
            spaceBefore=12
        )
        
        # Title
        story.append(Paragraph("ISO 9283 Robot Certification Report", title_style))
        story.append(Spacer(1, 0.2*inch))
        
        # Certificate ID
        story.append(Paragraph(f"Certificate ID: {certificate.metadata.certificate_id}", styles['Normal']))
        story.append(Paragraph(f"Issue Date: {certificate.metadata.issue_date}", styles['Normal']))
        story.append(Paragraph(f"Valid Until: {certificate.metadata.valid_until}", styles['Normal']))
        story.append(Paragraph(f"Laboratory: {certificate.metadata.laboratory}", styles['Normal']))
        story.append(Paragraph(f"Technician: {certificate.metadata.technician}", styles['Normal']))
        story.append(Spacer(1, 0.2*inch))
        
        # Hashes
        story.append(Paragraph("Configuration Integrity", heading_style))
        story.append(Paragraph(f"Dataset Hash: {certificate.metadata.dataset_hash[:16]}...", styles['Normal']))
        story.append(Paragraph(f"Robot Model Hash: {certificate.metadata.robot_model_hash[:16]}...", styles['Normal']))
        story.append(Paragraph(f"SRDF Hash: {certificate.metadata.srdf_hash[:16]}...", styles['Normal']))
        story.append(Spacer(1, 0.2*inch))
        
        # Robot Information
        story.append(Paragraph("Robot Under Test", heading_style))
        robot_data = [
            ["Manufacturer:", certificate.robot.manufacturer],
            ["Model:", certificate.robot.model],
            ["Serial Number:", certificate.robot.serial_number],
            ["Software Version:", certificate.robot.software_version],
            ["Payload:", f"{certificate.robot.payload_kg} kg"],
            ["Reach:", f"{certificate.robot.reach_m} m"]
        ]
        robot_table = Table(robot_data, colWidths=[2*inch, 4*inch])
        robot_table.setStyle(TableStyle([
            ('GRID', (0,0), (-1,-1), 1, colors.grey),
            ('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
            ('PADDING', (0,0), (-1,-1), 6)
        ]))
        story.append(robot_table)
        story.append(Spacer(1, 0.2*inch))
        
        # Test Conditions
        story.append(Paragraph("Test Conditions", heading_style))
        condition_data = [
            ["Temperature:", f"{certificate.test_conditions.ambient_temperature_c} °C"],
            ["Humidity:", f"{certificate.test_conditions.relative_humidity_percent} %"],
            ["Measurement System:", certificate.test_conditions.measurement_system],
            ["Calibration Date:", certificate.test_conditions.measurement_system_calibration_date],
            ["Test Cube:", f"{certificate.test_conditions.test_cube_size_mm} mm"],
            ["Repetitions per Pose:", str(certificate.test_conditions.repetitions_per_pose)],
            ["Pose Strategy:", certificate.test_conditions.pose_strategy],
            ["Outlier Threshold:", f"{certificate.test_conditions.outlier_threshold}σ"],
            ["Confidence Level:", f"{certificate.test_conditions.confidence_level*100}%"]
        ]
        condition_table = Table(condition_data, colWidths=[2.5*inch, 3.5*inch])
        condition_table.setStyle(TableStyle([
            ('GRID', (0,0), (-1,-1), 1, colors.grey),
            ('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
            ('PADDING', (0,0), (-1,-1), 6)
        ]))
        story.append(condition_table)
        story.append(Spacer(1, 0.2*inch))
        
        # Results by Pose
        story.append(Paragraph("Results by Pose", heading_style))
        result_data = [["Pose", "Accuracy (mm)", "Repeatability (mm)", "Samples", "Outliers"]]
        for pose in certificate.pose_results:
            result_data.append([
                pose.pose_label,
                f"{pose.accuracy_mm:.3f}",
                f"{pose.repeatability_mm:.3f}",
                str(pose.num_samples),
                str(pose.outliers_removed)
            ])
        
        result_table = Table(result_data)
        result_table.setStyle(TableStyle([
            ('GRID', (0,0), (-1,-1), 1, colors.grey),
            ('BACKGROUND', (0,0), (-1,0), colors.lightgrey),
            ('ALIGN', (1,0), (-1,-1), 'CENTER'),
            ('PADDING', (0,0), (-1,-1), 6)
        ]))
        story.append(result_table)
        story.append(Spacer(1, 0.2*inch))
        
        # Overall Results
        story.append(Paragraph("Overall Certification", heading_style))
        overall_data = [
            ["Mean Accuracy:", f"{certificate.overall.mean_accuracy_mm:.3f} mm"],
            ["Max Accuracy:", f"{certificate.overall.max_accuracy_mm:.3f} mm"],
            ["Accuracy 95% CI:", f"±{certificate.overall.accuracy_95_confidence_mm:.3f} mm"],
            ["Mean Repeatability:", f"{certificate.overall.mean_repeatability_mm:.3f} mm"],
            ["Max Repeatability:", f"{certificate.overall.max_repeatability_mm:.3f} mm"],
            ["Repeatability 95% CI:", f"±{certificate.overall.repeatability_95_confidence_mm:.3f} mm"],
            ["Status:", certificate.overall.certification_status]
        ]
        
        # Color code status
        status_color = colors.green
        if certificate.overall.certification_status == "CONDITIONAL":
            status_color = colors.orange
        elif certificate.overall.certification_status == "FAIL":
            status_color = colors.red
            
        overall_table = Table(overall_data, colWidths=[2.5*inch, 3.5*inch])
        overall_table.setStyle(TableStyle([
            ('GRID', (0,0), (-1,-1), 1, colors.grey),
            ('BACKGROUND', (0,0), (0,-1), colors.lightgrey),
            ('TEXTCOLOR', (1,-2), (1,-2), status_color),
            ('PADDING', (0,0), (-1,-1), 6)
        ]))
        story.append(overall_table)
        
        # Issues if any
        if certificate.metadata.issues:
            story.append(Spacer(1, 0.2*inch))
            story.append(Paragraph("Issues / Warnings", heading_style))
            for issue in certificate.metadata.issues:
                story.append(Paragraph(f"• {issue}", styles['Normal']))
        
        # Build PDF
        doc.build(story)