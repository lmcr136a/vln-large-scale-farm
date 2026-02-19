#!/usr/bin/env python3
import time, math, random

WRITER = "/dev/pts/4"     # set this
RATE_HZ = 10

# base near your example (34 deg, -118 deg)
base_lat = 34.068978
base_lon = -118.444621
alt_m = 115.56

def nmea_checksum(body: str) -> str:
    c = 0
    for ch in body:
        c ^= ord(ch)
    return f"{c:02X}"

def dd_to_nmea_lat(lat):
    hemi = "N" if lat >= 0 else "S"
    lat = abs(lat)
    deg = int(lat)
    minutes = (lat - deg) * 60.0
    return f"{deg:02d}{minutes:07.4f}", hemi

def dd_to_nmea_lon(lon):
    hemi = "E" if lon >= 0 else "W"
    lon = abs(lon)
    deg = int(lon)
    minutes = (lon - deg) * 60.0
    return f"{deg:03d}{minutes:07.4f}", hemi

def make(sentence_body: str) -> str:
    return f"${sentence_body}*{nmea_checksum(sentence_body)}"

t = 0.0
with open(WRITER, "w") as f:
    while True:
        t += 0.05

        # tiny circular motion + noise
        lat = base_lat + 10000 * 0.00001 * math.cos(t) + random.gauss(0, 0.000002)
        lon = base_lon + 10000 * 0.00001 * math.sin(t) + random.gauss(0, 0.000002)

        lat_nmea, lat_hemi = dd_to_nmea_lat(lat)
        lon_nmea, lon_hemi = dd_to_nmea_lon(lon)

        # time field HHMMSS.ss (fake but formatted)
        hhmmss = time.strftime("%H%M%S", time.gmtime())
        hhmmss_ss = f"{hhmmss}.00"

        # GGA (fix quality 2 like your sample)
        gga_body = f"GNGGA,{hhmmss_ss},{lat_nmea},{lat_hemi},{lon_nmea},{lon_hemi},2,12,1.47,{alt_m:.3f},M,-32.846,M,,0000"
        rmc_body = f"GNRMC,{hhmmss_ss},A,{lat_nmea},{lat_hemi},{lon_nmea},{lon_hemi},0.091,,100226,,,D,V"
        vtg_body = "GNVTG,,T,,M,0.091,N,0.168,K,D"
        gsa_body = "GNGSA,A,3,25,12,29,28,05,46,44,18,21,,,,2.25,1.47,1.69,1"

        print(gga_body)

        # Some “extra” satellite view noise like your logs (not exact, but realistic)
        gsv_body = "GPGSV,1,1,04,18,20,247,26,21,46,069,19,28,12,290,24,29,49,319,26"

        # Occasionally inject garbage to mimic real stream glitches
        if random.random() < 0.03:
            f.write("D`)&S_ӱi*\r\n")
        
        for body in (gsv_body, rmc_body, vtg_body, gga_body, gsa_body):
            f.write(make(body) + "\r\n")

        f.flush()
        time.sleep(1.0 / RATE_HZ)
