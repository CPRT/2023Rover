from typing import Set

SORTED_BY_SIZE = "sorted_by_size"
YAW_TO_CONTOUR = "yaw_to_contour"
REJECT = "reject"
FAR = "far"
CLOSE = "close"

class DetectionTemp:
    def __init__(self, unique_object_id_str: str):
        self.unique_object_id = unique_object_id_str

def is_contour_size_index(index: int, unique_label: str) -> bool:
    return f"sorted_by_size{index}*" in unique_label

def get_contour_yaw_by_size_index(index: int, detections) -> float:
    for det in detections:
        if is_contour_size_index(index, det.unique_object_id):
            return get_yaw_from_label(det.unique_object_id)
        
    return -360

def get_yaw_from_label(unique_label: str) -> bool:
    try:
        start_index = unique_label.index("yaw_to_contour")+len("yaw_to_contour")
        substring = unique_label[start_index:]
        end_index = substring.index("*") + len(unique_label) - len(substring)
        return float(unique_label[start_index:end_index])
    except Exception as e:
        return -360

def get_contour_size_index(unique_label: str) -> int:
    # try:
    return float(unique_label[unique_label.index("sorted_by_size")+len("sorted_by_size"):unique_label.index("*")])
    # except Exception as e:
    #     return -1

def get_marker_id_from_label(unique_label: str) -> int:
    # try:
    return int(unique_label[unique_label.index("-ArucoID")+len("-ArucoID"):unique_label.index("*")])
    # except Exception as e:
    #     return -1

def unique_object_id_from_tags(base: str, tags: Set[str]) -> str:
    if REJECT in tags:
        return ""
    
    tag_str = ""
    
    if MathStep.CoreTag.FAR in tags:
        tag_str += f"-{FAR}"
    elif MathStep.CoreTag.CLOSE in tags:
        tag_str += f"-{CLOSE}"
    else:
        return "" # Error

    for tag in tags:
        if not isinstance(tag, str):
            continue
        if SORTED_BY_SIZE in tag:
            tag_str += "-" + str(tag)
        if YAW_TO_CONTOUR in tag:
            tag_str += "-" + str(tag)
    
    return base + tag_str



def main():
    lst = [
        DetectionTemp("BlueLED-0-close-sorted_by_size1*-yaw_to_contour23.20*"), 
        DetectionTemp("BlueLED-2-far-yaw_to_contour16.61*-sorted_by_size2*")
    ]

    print(get_contour_yaw_by_size_index(1, lst))
    print(get_contour_yaw_by_size_index(2, lst))

if __name__ == "__main__":
    main()