import argparse
from colour_processing import ColourProcessing
from mask_step import MaskStep
from datatypes import HSVRange, HSV
from cv2_helper import BGR, HUE_MAX, SATURATION_MAX, VALUE_MAX
from file_operations import MultipleImages

from simple_mask_steps import SingleChannelRaiseSaturationStep, ThresholdMaskStep, EqualizeHistStep, ErodeDilateStep
from hsv_range_mask_step import HSVRangeMaskStep

def main():
    parser = argparse.ArgumentParser(
        prog="Colour Processing Images",
        description="Processes images following multiple mask steps with tuning features")
    
    parser.add_argument('-i', '--image')
    parser.add_argument('-f', '--folder')
    parser.add_argument('-s', '--scale')
    parser.add_argument('-d', '--displayScaling')
    parser.add_argument('-t', '--tune', action='store_true')
    parser.add_argument('--cprtTrailsBlue', action='store_true')
    parser.add_argument('--cprtTrailsRed', action='store_true')

    args = parser.parse_args()

    colour_processing: ColourProcessing = None
    image_scaling: float = 1.0
    display_scaling: float = 1.0

    if args.scale and args.scale is not None:
        try:
            image_scaling = float(args.scale)
        except Exception as e:
            print(f"-s {image_scaling}   must be a float (Thrown exception: {e})")

    if args.displayScaling and args.displayScaling is not None:
        try:
            display_scaling = float(args.displayScaling)
        except Exception as e:
            print(f"-s {display_scaling}   must be a float (Thrown exception: {e})")

    if args.cprtTrailsBlue:
        colour_processing = ColourProcessing(
            image_scaling,
            display_scaling,
            tuple([
                HSVRangeMaskStep('Step 1 - HSV', HSVRange(HSV(100, 159, 171), HSV(118, 255, 255)), return_mask=True), 
                ErodeDilateStep('Step 4 - Erode Dilate', 5, 12),
            ])
        )

    elif args.cprtTrailsRed:
        colour_processing = ColourProcessing(
            image_scaling,
            display_scaling,
            tuple([
                HSVRangeMaskStep('Step 1 - HSV', HSVRange(HSV(0, 74, 88), HSV(176, 255, 255)), return_mask=True, invert_hue=True), 
                ErodeDilateStep('Step 4 - Erode Dilate', 10, 33), 
            ])
        )

    if args.image and args.image is not None:
        if args.tune:
            colour_processing.single_image_mask_tuning(args.image)
        else:
            colour_processing.single_image_mask(args.image)

    if args.folder and args.folder is not None:
        images = MultipleImages()
        images.add_from_directory(args.folder)
        if args.tune:
            colour_processing.multi_image_mask_tuning(images.get_next_image)

if __name__ == "__main__":
    main()