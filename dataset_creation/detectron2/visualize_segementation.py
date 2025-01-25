import cv2
import numpy as np
import os

DATASET_PATH = "/workspace/Datasets/docking_3_synced"
RUN_NAME = "run1"
MAX_LABELS = 134
np.random.seed(40)


# def generate_colour_code(num_classes):
#     """
#     Generate a list of unique colors for the number of classes.
#     """
#     id_to_colour = {}
#     for i in range(num_classes):
#         # Ensure somewhat distinct colors by using more controlled randomization
#         hue = (i * 255 // num_classes) % 255
#         color = cv2.cvtColor(np.uint8([[[hue, 255, 255]]]), cv2.COLOR_HSV2BGR)[0][0]
#         id_to_colour[i] = color.tolist()
#     return id_to_colour


def generate_colour_code(num_classes):
    """
    Generate a list of colours for the number of classes.
    """
    id_to_colour = {}
    for i in range(num_classes):
        id_to_colour[i] = np.random.choice(range(255), size=3)
    return id_to_colour


def visualize_segmentation(image, prediction, id_color_map):
    """
    Overlay segmentation masks on the original image with color coding.
    """
    # Create a colored overlay for all unique labels at once
    overlay = image.copy()
    for label, color in id_color_map.items():
        mask = prediction == label
        if mask.any():  # Only process if label exists in prediction
            overlay[mask] = color

    # Blend original and colored overlay
    blended = cv2.addWeighted(image, 0.3, overlay, 0.7, 0)
    return blended


def main():
    # Check if directory exists
    if not os.path.exists(os.path.join(DATASET_PATH, RUN_NAME)):
        raise FileNotFoundError(
            f"Directory '{os.path.join(DATASET_PATH, RUN_NAME)}' not found."
        )
    else:
        print(f"Visualizing segmentation for '{RUN_NAME}'.")

    # Gather Images and Predictions
    images = [
        f
        for f in os.listdir(os.path.join(DATASET_PATH, RUN_NAME))
        if f.endswith("_color.png")
    ]
    images.sort()
    predictions = [
        f
        for f in os.listdir(os.path.join(DATASET_PATH, RUN_NAME))
        if f.endswith("_predicted.png")
    ]
    predictions.sort()

    # Generate color map
    id_color_map = generate_colour_code(MAX_LABELS)

    # Process each image-prediction pair
    for img_name, pred_name in zip(images, predictions):
        # Load images
        image = cv2.imread(os.path.join(DATASET_PATH, RUN_NAME, img_name))
        prediction = cv2.imread(
            os.path.join(DATASET_PATH, RUN_NAME, pred_name), cv2.IMREAD_GRAYSCALE
        )

        # Visualize segmentation
        result = visualize_segmentation(image, prediction, id_color_map)

        # Display
        cv2.imshow("Segmentation Visualization", result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
