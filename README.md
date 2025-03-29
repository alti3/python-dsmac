# Python DSMAC Demo for GPS-Denied Drone Localization

This repository contains a Python script (`dsmac.py`) demonstrating a basic approach to drone localization without GPS by matching the drone's camera footage to a georeferenced satellite image. This technique is inspired by Digital Scene Matching Area Correlator (DSMAC) systems.

The script calculates the drone's estimated latitude and longitude and outputs it as a mock NMEA GPGGA sentence, simulating the output of a GPS device.

## How it Works

1.  **Image Loading:** Loads a georeferenced satellite image (`satellite.jpg`) and a drone camera image (`drone.jpg`).
2.  **Feature Detection & Matching:** Uses OpenCV's ORB (Oriented FAST and Rotated BRIEF) feature detector to find keypoints and descriptors in both images. It then matches these features using a Brute-Force Hamming matcher.
3.  **Homography Calculation:** Computes a homography matrix (using RANSAC) that maps points from the drone image plane to the satellite image plane based on the best feature matches.
4.  **Coordinate Transformation:** Transforms the center point of the drone image to its corresponding pixel coordinate in the satellite image using the calculated homography.
5.  **Georeferencing:** Converts the satellite image pixel coordinate to geographic coordinates (latitude, longitude) using a linear interpolation based on the known bounding box coordinates of the satellite image.
6.  **NMEA Sentence Generation:** Formats the calculated latitude and longitude into a standard NMEA GPGGA sentence, including a calculated checksum. This simulates the output of a GPS receiver.
7.  **Output:** Prints the calculated coordinates and the NMEA sentence to the console and appends the NMEA sentence to a file named `nmea_output.csv`.

## Requirements

*   Python 3.x
*   OpenCV (`opencv-python`)
*   NumPy (`numpy`)

You can install the required libraries using pip:
```bash
pip install opencv-python numpy
```

## Usage

1.  **Prepare Images:**
    *   Place your georeferenced satellite image in the same directory as the script and name it `satellite.jpg`.
    *   Place your drone camera image in the same directory and name it `drone.jpg`.
2.  **Update Georeference Data:**
    *   Open `dsmac.py` and modify the following variables within the `main()` function with the correct bounding box coordinates for your `satellite.jpg`:
        ```python
        lat_top = 40.0000    # northern (top) latitude
        lon_left = -74.0000  # western (left) longitude
        lat_bottom = 39.5000 # southern (bottom) latitude
        lon_right = -73.5000 # eastern (right) longitude
        ```
3.  **Run the Script:**
    ```bash
    python dsmac.py
    ```

## Output

The script will print the computed latitude, longitude, and the generated NMEA GPGGA sentence to the console.

Example Console Output:
```
Computed GPS location from DSMAC matching:
Latitude: 39.751234, Longitude: -73.749876
Mock NMEA GPGGA sentence:
$GPGGA,123456.78,3945.0740,N,07344.9926,W,1,08,0.9,100.0,M,0.0,M,,*XX
```
*(Note: Timestamp and checksum will vary)*

Additionally, each generated NMEA sentence will be appended to the `nmea_output.csv` file in the same directory.

## Limitations & Notes

*   **Basic Implementation:** This is a simplified demonstration. Real-world DSMAC systems use significantly more sophisticated algorithms for correlation, matching, filtering, and handling variations in lighting, perspective, and scale.
*   **Feature Matching Robustness:** ORB is relatively fast but may not be robust enough for all scenarios (e.g., significant changes in viewpoint, lighting, or low-texture areas). Consider exploring other feature detectors like SIFT or SURF (though note potential patent restrictions) or deep learning-based methods for better results.
*   **Linear Georeferencing:** The `pixel_to_latlon` function assumes a simple linear relationship between pixel coordinates and geographic coordinates. This is often an approximation, especially for large areas or images with significant distortion. Accurate georeferencing might require more complex map projections.
*   **Performance:** Processing is done frame-by-frame on static images. Real-time application would require integration with a camera feed and optimization for continuous processing.
*   **Altitude:** The altitude in the NMEA sentence is currently hardcoded (`altitude=100`). A real system would need a way to estimate or obtain altitude (e.g., barometer, triangulation if possible).

## Potential Extensions

*   Integrate with a live drone camera feed (e.g., using `cv2.VideoCapture`).
*   Implement Kalman filtering or other state estimation techniques to smooth the location output and handle potential matching failures.
*   Use more advanced feature matching or template matching algorithms.
*   Send the NMEA data over a serial port (real or virtual) using `pyserial` to interface with other navigation software.
