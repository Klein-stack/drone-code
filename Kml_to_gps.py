import xml.etree.ElementTree as ET
import os

def kml_file_processor(file_path):
   
    tree = ET.parse(file_path)
    root = tree.getroot()
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    coordinates = []
    for coord in root.findall('.//kml:coordinates', ns):
        for line in coord.text.strip().split():
            parts = line.split(',')
            lon = float(parts[0])
            lat = float(parts[1])
            alt = float(parts[2]) if len(parts) > 2 else None
            coordinates.append((lon, lat, alt) if alt is not None else (lon, lat))
    return coordinates

# Example usage:
file_path = r"Sargam_stage.kml"  # Replace with your KML file path
coords = kml_file_processor(file_path)
print(coords)

# Store coordinates in a text file in the same directory as this script
output_path = os.path.join(os.path.dirname(__file__), "coordinates.txt")
with open(output_path, "w") as f:
    for coord in coords:
        f.write(f"{coord}\n")
print(f"Coordinates saved to {output_path}")