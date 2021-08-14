#!/usr/bin/env python3
from pykml import parser
import os
import xmltodict
import json

def parse_kml(file_):
    cur_path = os.path.dirname(__file__)
    with open("{}/{}".format(cur_path,file_)) as xml_file:
        kml_dict = xmltodict.parse(xml_file.read())
        kmljson = json.loads(json.dumps(kml_dict))
        kmljson_dict = kmljson["kml"]["Document"]
        print(kmljson_dict)
       
        poly = kmljson_dict["Placemark"][0]["Polygon"]["outerBoundaryIs"]["LinearRing"]["coordinates"]
        
        """
        for i in range (1,len(kmljson_dict["features"])):
            #print(kmljson_dict["features"][i]["geometry"]["type"])
            points.append(kmljson_dict["features"][i]["geometry"]["coordinates"])
        #print(points)
        return points,poly
        """

def parse_json(file_,req):
    points = []
    nfzones = []
    cur_path = os.path.dirname(__file__)
    with open("{}/{}".format(cur_path,file_)) as f:
        kmljson_dict = json.load(f)
        if req == "WAYPOINT":
            poly = kmljson_dict["features"][0]["geometry"]["coordinates"][0]
            for point in range (1,len(kmljson_dict["features"])):
                #print(kmljson_dict["features"][i]["geometry"]["type"])
                points.append(kmljson_dict["features"][point]["geometry"]["coordinates"])
            #print(points)
            return points,poly
        if req == "ZONE":
            for nfzone in range (0,len(kmljson_dict["features"])):
                nfzones.append(kmljson_dict["features"][0]["geometry"]["coordinates"][0])
            return nfzones


"""
[
    [[73.7709701527201, 18.5190981318863, 611.977265987933], [73.7697229373043, 18.5181845734462, 616.906371930345], [73.769293108104, 18.5167295962605, 622.147709150573], [73.7707161899849, 18.5164707474778, 618.296192976349], [73.7719641901693, 18.5177229853325, 611.175953972266], [73.7709701527201, 18.5190981318863, 611.977265987933]], 
    [[73.7709701527201, 18.5190981318863, 611.977265987933], [73.7697229373043, 18.5181845734462, 616.906371930345], [73.769293108104, 18.5167295962605, 622.147709150573], [73.7707161899849, 18.5164707474778, 618.296192976349], [73.7719641901693, 18.5177229853325, 611.175953972266], [73.7709701527201, 18.5190981318863, 611.977265987933]], 
    [[73.7709701527201, 18.5190981318863, 611.977265987933], [73.7697229373043, 18.5181845734462, 616.906371930345], [73.769293108104, 18.5167295962605, 622.147709150573], [73.7707161899849, 18.5164707474778, 618.296192976349], [73.7719641901693, 18.5177229853325, 611.175953972266], [73.7709701527201, 18.5190981318863, 611.977265987933]], 
    [[73.7709701527201, 18.5190981318863, 611.977265987933], [73.7697229373043, 18.5181845734462, 616.906371930345], [73.769293108104, 18.5167295962605, 622.147709150573], [73.7707161899849, 18.5164707474778, 618.296192976349], [73.7719641901693, 18.5177229853325, 611.175953972266], [73.7709701527201, 18.5190981318863, 611.977265987933]], 
    [[73.7709701527201, 18.5190981318863, 611.977265987933], [73.7697229373043, 18.5181845734462, 616.906371930345], [73.769293108104, 18.5167295962605, 622.147709150573], [73.7707161899849, 18.5164707474778, 618.296192976349], [73.7719641901693, 18.5177229853325, 611.175953972266], [73.7709701527201, 18.5190981318863, 611.977265987933]]
]
"""
if __name__ == "__main__":
    pol,poin = parse_json("google_map_waypoint1.geojson")
    pol,poin = parse_kml("google_map_waypoint1.kml")
    print(pol,poin)