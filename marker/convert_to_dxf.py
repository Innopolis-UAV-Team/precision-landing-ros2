"""
SVG to DXF Converter
This script converts SVG files containing rectangles to DXF format, primarily for laser cutting applications.
It specifically handles SVG rectangles and preserves their dimensions and positions in the output DXF file.

Features:
- Parses SVG file with proper scaling and transformations
- Handles SVG viewBox and mm units
- Supports nested group transformations
- Creates DXF output with proper line entities
- Maintains original dimensions and positions

Usage:
Place this script in the same directory as your SVG file and update the svg_path variable.
The output will be saved as 'board_converted.dxf'.

Author: sainquake GPT
"""

from pathlib import Path
import xml.etree.ElementTree as ET  # For parsing SVG XML structure

svg_path = Path("board(5).svg")
tree = ET.parse(svg_path)
root = tree.getroot()

width_attr = root.attrib.get('width')
height_attr = root.attrib.get('height')
viewBox = root.attrib.get('viewBox')
vb_min_x, vb_min_y, vb_w, vb_h = [float(x) for x in viewBox.split()]

def parse_length_mm(s):
    """Convert SVG length to millimeters, handling units if present."""
    if s.endswith('mm'):
        return float(s[:-2])
    return float(s)

# Calculate scaling factors to convert from viewBox units to millimeters
width_mm = parse_length_mm(width_attr)
height_mm = parse_length_mm(height_attr)
sx = width_mm / vb_w  # Scale factor for X coordinates
sy = height_mm / vb_h  # Scale factor for Y coordinates

def parse_transform(t):
    """Parse SVG transform attribute to extract translation values.
    
    Args:
        t (str): SVG transform attribute string
        
    Returns:
        tuple: (tx, ty) translation values in x and y directions
    """
    if not t:
        return 0.0, 0.0
    part = t.split(')')[0] + ')'  # Get first transform command
    if 'translate' in part:
        # Extract values from translate(x,y) or translate(x)
        inside = part[part.find('(')+1:part.find(')')].replace(',', ' ').split()
        if len(inside)==1:
            tx = float(inside[0]); ty = 0.0  # Only X translation provided
        else:
            tx, ty = map(float, inside[:2])  # Both X and Y translations
        return tx, ty
    return 0.0, 0.0  # No translation found

# List to store all rectangles found in the SVG
rects = []

def walk(elem, acc_tx=0.0, acc_ty=0.0):
    """Recursively walk through SVG elements to find and process rectangles.
    
    Args:
        elem: Current XML element being processed
        acc_tx (float): Accumulated X translation from parent transforms
        acc_ty (float): Accumulated Y translation from parent transforms
    """
    # Get transform for current element and add to accumulated transform
    tx, ty = parse_transform(elem.attrib.get('transform', ''))
    acc_tx += tx; acc_ty += ty
    
    for child in list(elem):
        if child.tag.endswith('rect'):
            # Found a rectangle - extract its properties with accumulated transforms
            x = float(child.attrib.get('x', '0')) + acc_tx
            y = float(child.attrib.get('y', '0')) + acc_ty
            w = float(child.attrib.get('width'))
            h = float(child.attrib.get('height'))
            rects.append((x, y, w, h))
        else:
            # Recurse into non-rectangle elements
            walk(child, acc_tx, acc_ty)

walk(root)

def add_line(lines, x1, y1, x2, y2, layer="0"):
    """Add a line entity to the DXF content.
    
    Args:
        lines (list): List of DXF content lines
        x1, y1 (float): Start point coordinates
        x2, y2 (float): End point coordinates
        layer (str): DXF layer name (default: "0")
    """
    # Add DXF LINE entity with proper formatting for coordinates
    lines.extend(["0","LINE","8",layer,"10",f"{x1:.6f}","20",f"{y1:.6f}","11",f"{x2:.6f}","21",f"{y2:.6f}"])

def rect_to_coords(x, y, w, h):
    """Convert SVG rectangle coordinates to DXF coordinates in millimeters.
    
    Args:
        x, y (float): Top-left corner of rectangle in SVG coordinates
        w, h (float): Width and height of rectangle
        
    Returns:
        tuple: Four corner points of rectangle in DXF coordinates ((x0,y0), (x1,y0), (x1,y1), (x0,y1))
    """
    # Convert from SVG viewBox coordinates to millimeters
    x0_mm = (x - vb_min_x) * sx
    y0_mm = (y - vb_min_y) * sy
    x1_mm = (x + w - vb_min_x) * sx
    y1_mm = (y + h - vb_min_y) * sy
    
    # Flip Y coordinates since DXF has origin at bottom-left
    y0 = height_mm - y0_mm
    y1 = height_mm - y1_mm
    
    return (x0_mm, y0), (x1_mm, y0), (x1_mm, y1), (x0_mm, y1)

# Initialize DXF file content with header
content = ["0","SECTION","2","ENTITIES"]

# Convert each rectangle to four lines in the DXF file
for (x,y,w,h) in rects:
    # Get the four corners of the rectangle
    p1, p2, p3, p4 = rect_to_coords(x,y,w,h)
    # Add four lines to create the rectangle
    add_line(content, *p1, *p2)  # Top line
    add_line(content, *p2, *p3)  # Right line
    add_line(content, *p3, *p4)  # Bottom line
    add_line(content, *p4, *p1)  # Left line

# Add DXF file footer
content.extend(["0","ENDSEC","0","EOF"])

# Write the DXF file
dxf_path = Path("board_converted.dxf")
with open(dxf_path, "w") as f:
    f.write("\n".join(content))

# Return the path and number of rectangles processed
str(dxf_path), len(rects)
