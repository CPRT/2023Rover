import argparse
import yaml
import cv2
from colour_processing import ColourProcessing
from file_operations import MultipleImages


def main():
    parser = argparse.ArgumentParser(
        prog="Colour Processing Images",
        description="Processes images following multiple mask steps with tuning features")
    
    parser.add_argument('-i', '--image')
    parser.add_argument('-f', '--folder')
    parser.add_argument('-s', '--scale')
    parser.add_argument('-d', '--displayScaling')
    parser.add_argument('-t', '--tune', action='store_true')
    parser.add_argument('-b', '--cprtTrailsBlue')
    parser.add_argument('-r', '--cprtTrailsRed')
    parser.add_argument('--cprtTrailsIR')

    args = parser.parse_args()

    colour_processing: ColourProcessing = None
    image_scaling: float = 1.0
    display_scaling: float = 1.0

    if args.scale and args.scale is not None:
        try:
            image_scaling = float(args.scale)
        except Exception as e:
            print(f"-s {image_scaling}   must be a float (raised exception: {e})")

    if args.displayScaling is not None:
        try:
            display_scaling = float(args.displayScaling)
        except Exception as e:
            print(f"-s {display_scaling}   must be a float (raised exception: {e})")

    if args.cprtTrailsBlue is not None:
        with open(args.cprtTrailsBlue) as yml:
            parameters = yaml.safe_load(yml)
            colour_processing = ColourProcessing.from_string(parameters['/**']['ros__parameters']['blue_led'])
            if isinstance(colour_processing, str):
                raise ValueError(f"Failed to load ColourProcessing for blue_led. Error: {colour_processing}")
                
    elif args.cprtTrailsRed is not None:
        with open(args.cprtTrailsRed) as yml:
            parameters = yaml.safe_load(yml)
            colour_processing = ColourProcessing.from_string(parameters['/**']['ros__parameters']['red_led'])
            if isinstance(colour_processing, str):
                raise ValueError(f"Failed to load ColourProcessing for red_led. Error: {colour_processing}")

    elif args.cprtTrailsIR is not None:
        with open(args.cprtTrailsIR) as yml:
            parameters = yaml.safe_load(yml)
            colour_processing = ColourProcessing.from_string(parameters['/**']['ros__parameters']['ir_led'])
            if isinstance(colour_processing, str):
                raise ValueError(f"Failed to load ColourProcessing for ir_led. Error: {colour_processing}")

    if args.image and args.image is not None:
        if args.tune:
            colour_processing.single_image_process_tuning(args.image)
        else:
            colour_processing.single_image_process(cv2.imread(args.image))

    if args.folder and args.folder is not None:
        images = MultipleImages()
        images.add_from_directory(args.folder)
        if args.tune:
            colour_processing.multi_image_process_tuning(images.get_next_image)
        else:
            for image in images.as_list():
                colour_processing.single_image_process(image)
            

if __name__ == "__main__":
    main()