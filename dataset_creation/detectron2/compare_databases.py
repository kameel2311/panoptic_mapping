"""Very Naive Script to Compare Databases"""

import cv2
import numpy as np
import os
import json
import pandas as pd
import itertools
from tqdm import tqdm
import matplotlib.pyplot as plt

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


def create_instance_to_class_map(labels):
    """
    Create a mapping from instance IDs to class IDs.
    """
    instance_to_class_map = {}
    for label in labels:
        instance_to_class_map[label["id"]] = label["category_id"]
    # Adding the Unknown class
    instance_to_class_map[0] = 0
    return instance_to_class_map


def get_id_label_map(dataframe, class_col="ClassID", name_col="Name"):
    """
    Create a mapping from class IDs to class names.
    """
    class_id_to_name_map = {}
    class_ids = dataframe[class_col].values
    class_names = dataframe[name_col].values
    for class_id, class_name in zip(class_ids, class_names):
        class_id_to_name_map[class_id] = class_name
    return class_id_to_name_map


def get_class_count(class_count):
    """
    Count the number of instances of each class in the dataset.
    """
    class_count = list(itertools.chain.from_iterable(class_count))
    classes = np.unique(class_count)
    database = {class_id: class_count.count(class_id) for class_id in classes}
    return database


def plot_metadata_both(
    flat_instance_frame_count,
    flat_class_frame_count,
    warehouse_instance_frame_count,
    warehouse_class_frame_count,
):
    fig, ax = plt.subplots(2, 1, figsize=(10, 10))
    ax[0].plot(flat_instance_frame_count, label="Flat", color="blue", alpha=0.5)
    ax[0].plot(
        warehouse_instance_frame_count, label="Warehouse", color="red", alpha=0.5
    )
    ax[0].set_title("Count of Instances per Frame")
    ax[0].set_xlabel("Frame")
    ax[0].set_ylabel("Count")
    ax[0].grid()
    ax[0].legend()
    ax[1].plot(flat_class_frame_count, label="Flat", color="blue", alpha=0.5)
    ax[1].plot(warehouse_class_frame_count, label="Warehouse", color="red", alpha=0.5)
    ax[1].set_title("Count of Classes per Frame")
    ax[1].set_xlabel("Frame")
    ax[1].set_ylabel("Count")
    ax[1].grid()
    ax[1].legend()
    plt.suptitle("Flat vs Warehouse Class and Instance Counts")
    plt.show()


DATASET_DIR = "/workspace/Datasets"
MAX_LABELS = 134
VISUALIZE_CLASS = True  # Else Coloured by Instance
VISUALIZE = False
np.random.seed(40)


def main():
    DATASET_NAME = "flat_dataset"
    DATASET_PATH = os.path.join(DATASET_DIR, DATASET_NAME)
    RUN_NAME = "run1"
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
    labels = [
        f
        for f in os.listdir(os.path.join(DATASET_PATH, RUN_NAME))
        if f.endswith("_labels.json")
    ]
    labels.sort()

    # Load Detection Labels
    detection_metadata = pd.read_csv(os.path.join(DATASET_PATH, "detectron_labels.csv"))

    # Generate color map
    id_color_map = generate_colour_code(MAX_LABELS)
    classID_to_name = get_id_label_map(detection_metadata)

    # Per Frame Metadata
    per_frame_instances = []
    per_frame_classes = []
    class_count = []

    # Process each image-prediction pair
    for img_name, pred_name, label_name in tqdm(zip(images, predictions, labels)):
        # Load images
        image = cv2.imread(os.path.join(DATASET_PATH, RUN_NAME, img_name))
        prediction = cv2.imread(
            os.path.join(DATASET_PATH, RUN_NAME, pred_name), cv2.IMREAD_GRAYSCALE
        )

        # Load JSON labels
        with open(os.path.join(DATASET_PATH, RUN_NAME, label_name), "r") as file:
            label = json.load(file)
            instance_to_class = create_instance_to_class_map(label)
            # print(instance_to_class)

        # Convert instance IDs to class IDs
        class_prediction = np.vectorize(instance_to_class.get)(prediction)
        classes_in_frame = np.unique(class_prediction)
        labels_in_frame = [classID_to_name[class_id] for class_id in classes_in_frame]
        # print(labels_in_frame)

        # Collect per frame metadata
        per_frame_instances.append(len(np.unique(prediction)))
        per_frame_classes.append(len(classes_in_frame))
        class_count.append(labels_in_frame)

        # Visualize segmentation
        if VISUALIZE:
            if VISUALIZE_CLASS:
                result = visualize_segmentation(image, class_prediction, id_color_map)
            else:
                result = visualize_segmentation(image, prediction, id_color_map)

            # Display
            cv2.imshow("Segmentation Visualization", result)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    flat_instances = per_frame_instances
    flat_classes = per_frame_classes
    # Plot Metadata

    DATASET_NAME = "docking_3_sync_aligned"
    DATASET_PATH = os.path.join(DATASET_DIR, DATASET_NAME)
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
    labels = [
        f
        for f in os.listdir(os.path.join(DATASET_PATH, RUN_NAME))
        if f.endswith("_labels.json")
    ]
    labels.sort()

    # Load Detection Labels
    detection_metadata = pd.read_csv(os.path.join(DATASET_PATH, "detectron_labels.csv"))

    # Generate color map
    id_color_map = generate_colour_code(MAX_LABELS)
    classID_to_name = get_id_label_map(detection_metadata)

    # Per Frame Metadata
    per_frame_instances = []
    per_frame_classes = []
    class_count = []

    # Process each image-prediction pair
    for img_name, pred_name, label_name in tqdm(zip(images, predictions, labels)):
        # Load images
        image = cv2.imread(os.path.join(DATASET_PATH, RUN_NAME, img_name))
        prediction = cv2.imread(
            os.path.join(DATASET_PATH, RUN_NAME, pred_name), cv2.IMREAD_GRAYSCALE
        )

        # Load JSON labels
        with open(os.path.join(DATASET_PATH, RUN_NAME, label_name), "r") as file:
            label = json.load(file)
            instance_to_class = create_instance_to_class_map(label)
            # print(instance_to_class)

        # Convert instance IDs to class IDs
        class_prediction = np.vectorize(instance_to_class.get)(prediction)
        classes_in_frame = np.unique(class_prediction)
        labels_in_frame = [classID_to_name[class_id] for class_id in classes_in_frame]
        # print(labels_in_frame)

        # Collect per frame metadata
        per_frame_instances.append(len(np.unique(prediction)))
        per_frame_classes.append(len(classes_in_frame))
        class_count.append(labels_in_frame)

    warehouse_instances = per_frame_instances
    warehouse_classes = per_frame_classes

    plot_metadata_both(
        flat_instances,
        flat_classes,
        warehouse_instances,
        warehouse_classes,
    )


if __name__ == "__main__":
    main()
