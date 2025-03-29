import cv2
import numpy as np
import datetime

def compute_homography(drone_img, sat_img):
    """
    Convert both images to grayscale, detect ORB keypoints/descriptors,
    match them using a brute-force matcher, and then compute a homography
    (using RANSAC) from the drone image to the satellite image.
    Returns the 3x3 homography matrix H and a mask of inliers.
    """
    # Convert images to grayscale
    gray_drone = cv2.cvtColor(drone_img, cv2.COLOR_BGR2GRAY)
    gray_sat = cv2.cvtColor(sat_img, cv2.COLOR_BGR2GRAY)
    
    # Create ORB detector (you can adjust the number of features)
    orb = cv2.ORB_create(1000)
    kp1, des1 = orb.detectAndCompute(gray_drone, None)
    kp2, des2 = orb.detectAndCompute(gray_sat, None)
    
    # Match descriptors using brute force Hamming distance
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)
    
    # Use the best 50 matches (or fewer if not available)
    num_matches = min(50, len(matches))
    if num_matches < 4:
        return None, None
    pts_drone = np.float32([kp1[m.queryIdx].pt for m in matches[:num_matches]]).reshape(-1,1,2)
    pts_sat   = np.float32([kp2[m.trainIdx].pt for m in matches[:num_matches]]).reshape(-1,1,2)
    
    # Compute homography using RANSAC
    H, mask = cv2.findHomography(pts_drone, pts_sat, cv2.RANSAC, 5.0)
    return H, mask

def pixel_to_latlon(x, y, img_width, img_height, lat_top, lon_left, lat_bottom, lon_right):
    """
    Convert a pixel coordinate (x, y) in the satellite image to a geographic
    coordinate using a linear mapping. This assumes the satellite image is north‐up.
    """
    lat = lat_top - (y / img_height) * (lat_top - lat_bottom)
    lon = lon_left + (x / img_width) * (lon_right - lon_left)
    return lat, lon

def decimal_to_nmea_lat(lat):
    """
    Convert a latitude in decimal degrees to NMEA format (ddmm.mmmm) along with
    a direction letter (N/S).
    """
    direction = 'N' if lat >= 0 else 'S'
    abs_lat = abs(lat)
    degrees = int(abs_lat)
    minutes = (abs_lat - degrees) * 60
    # Format as 2-digit degrees and 2-digit minutes with 4 decimals
    return f"{degrees:02d}{minutes:07.4f}", direction

def decimal_to_nmea_lon(lon):
    """
    Convert a longitude in decimal degrees to NMEA format (dddmm.mmmm) along with
    a direction letter (E/W).
    """
    direction = 'E' if lon >= 0 else 'W'
    abs_lon = abs(lon)
    degrees = int(abs_lon)
    minutes = (abs_lon - degrees) * 60
    return f"{degrees:03d}{minutes:07.4f}", direction

def calculate_nmea_checksum(nmea_str):
    """
    Compute the NMEA checksum by XOR’ing all characters between the '$' and '*'.
    """
    checksum = 0
    for char in nmea_str:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def create_gpgga_sentence(lat, lon, altitude=0):
    """
    Create an NMEA GPGGA sentence using the provided latitude, longitude, and altitude.
    Other fields such as fix quality, number of satellites, HDOP, and geoid separation
    are filled with dummy values.
    """
    now = datetime.datetime.utcnow()
    time_str = now.strftime("%H%M%S.%f")[:9]  # hhmmss.ss (up to hundredths)
    lat_str, lat_dir = decimal_to_nmea_lat(lat)
    lon_str, lon_dir = decimal_to_nmea_lon(lon)
    fix_quality = "1"  # GPS fix
    num_sat = "08"
    hdop = "0.9"
    altitude_str = f"{altitude:.1f}"
    geoid_sep = "0.0"
    
    # Build the sentence body (without the starting '$' and checksum)
    nmea_body = f"GPGGA,{time_str},{lat_str},{lat_dir},{lon_str},{lon_dir},{fix_quality},{num_sat},{hdop},{altitude_str},M,{geoid_sep},M,,"
    checksum = calculate_nmea_checksum(nmea_body)
    nmea_sentence = f"${nmea_body}*{checksum}"
    return nmea_sentence

def main():
    """
    Main function to demonstrate:
      1. Loading a satellite image (with known georeferencing parameters)
      2. Loading a drone camera image
      3. Matching the drone image to the satellite image to compute a homography
      4. Mapping the drone image center to a pixel coordinate in the satellite image,
         then converting that to latitude and longitude
      5. Creating and outputting an NMEA GPGGA sentence with the computed location
    """
    # Load your images (replace these filenames with your actual files)
    sat_img = cv2.imread("satellite.jpg")
    drone_img = cv2.imread("drone.jpg")
    
    if sat_img is None or drone_img is None:
        print("Error: Could not load images. Please ensure 'satellite.jpg' and 'drone.jpg' exist.")
        return
    
    # Satellite image georeference parameters (bounding box in decimal degrees)
    # (These values must be obtained from your satellite imagery metadata.)
    lat_top = 40.0000    # northern (top) latitude
    lon_left = -74.0000  # western (left) longitude
    lat_bottom = 39.5000 # southern (bottom) latitude
    lon_right = -73.5000 # eastern (right) longitude
    
    # Get satellite image dimensions
    sat_height, sat_width = sat_img.shape[:2]
    
    # Compute homography from the drone image to the satellite image
    H, mask = compute_homography(drone_img, sat_img)
    if H is None:
        print("Not enough matches found between images; cannot compute homography.")
        return
    
    # Use the center of the drone image as the reference point
    drone_height, drone_width = drone_img.shape[:2]
    center_drone = np.array([[drone_width / 2, drone_height / 2]], dtype=np.float32).reshape(-1, 1, 2)
    
    # Map the center point using the computed homography
    center_sat = cv2.perspectiveTransform(center_drone, H)
    center_sat_x, center_sat_y = center_sat[0, 0]
    
    # Convert the satellite pixel coordinate to geographic coordinates
    lat, lon = pixel_to_latlon(center_sat_x, center_sat_y, sat_width, sat_height,
                                 lat_top, lon_left, lat_bottom, lon_right)
    
    # Create an NMEA GPGGA sentence using the computed lat/lon (altitude here is set arbitrarily)
    nmea_sentence = create_gpgga_sentence(lat, lon, altitude=100)
    
    # Output the results
    print("Computed GPS location from DSMAC matching:")
    print(f"Latitude: {lat:.6f}, Longitude: {lon:.6f}")
    print("Mock NMEA GPGGA sentence:")
    print(nmea_sentence)
    
    # Optionally, you could feed this NMEA sentence to a serial port (using pyserial) to simulate a GPS device.
    # For example:
    # import serial
    # ser = serial.Serial('/dev/ttyUSB0', 9600)
    # ser.write(nmea_sentence.encode('ascii'))
    
    # For this demonstration, we also append the sentence to a CSV file.
    with open("nmea_output.csv", "a") as csvfile:
        csvfile.write(nmea_sentence + "\n")
    
    # You might also run this in a loop (or in response to new drone images) to continually update location.
    # For example:
    # while True:
    #     # acquire new drone_img from your camera feed
    #     # re-run matching and output the new NMEA sentence
    #     time.sleep(1)

if __name__ == "__main__":
    main()
