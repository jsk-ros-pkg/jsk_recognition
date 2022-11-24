#!/usr/bin/env python
# Copied from https://github.com/ageitgey/face_recognition/blob/master/examples/face_recognition_svm.py and then modified


import face_recognition
import rospy
import os
from sklearn import svm
from sklearn import neighbors
import math


class FaceRecognizer(object):
    """
    Face Recognizer Class with Extract features of faces with dlib and classify it with svm or knn.
    """

    def __init__(self, known_person_image_dir, fitting='svm'):
        """
        Image Directory
            <known_person_image_dir>/
                <person_1>/
                    <person_1_face-1>.jpg
                    <person_1_face-2>.jpg
                    .
                    .
                    <person_1_face-n>.jpg
                <person_2>/
                    <person_2_face-1>.jpg
                    <person_2_face-2>.jpg
                    .
                    .
                    <person_2_face-n>.jpg
                .
                .
                <person_n>/
                    <person_n_face-1>.jpg
                    <person_n_face-2>.jpg
                    .
                    .
                    <person_n_face-n>.jpg
        """

        encodings = []
        names = []

        person_names = os.listdir(known_person_image_dir)
        for person_name in person_names:
            person_dir = known_person_image_dir + '/' + person_name
            pictures = os.listdir(person_dir)
            for picture_file in pictures:
                fullpath = person_dir + '/' + picture_file
                person_picture = face_recognition.load_image_file(fullpath)
                face_bboxes = face_recognition.face_locations(
                    person_picture, model='cnn')
                face_encs = face_recognition.face_encodings(
                    person_picture, known_face_locations=face_bboxes, model='large')
                if len(face_bboxes) == 0:
                    rospy.logwarn('{}, contains {} persons. So this file is skipped.'.format(
                        person_name + '/' + picture_file, len(face_bboxes)))
                else:
                    print('{}, face_bboxes: {}'.format(
                        person_name + '/' + picture_file, face_bboxes))
                    index = face_bboxes.index(
                        max(face_bboxes, key=lambda x: (x[2]-x[0])*(x[3]-x[1])))
                    face_enc = face_encs[index]
                    encodings.append(face_enc)
                    names.append(person_name)

        self.fitting = fitting
        if self.fitting == 'svm':
            self.classifier = svm.SVC(gamma='scale', probability=True)
            self.classifier.fit(encodings, names)
        elif self.fitting == 'knn':
            n_neighbors = int(round(math.sqrt(len(encodings))))
            self.classifier = neighbors.KNeighborsClassifier(
                n_neighbors=n_neighbors, algorithm='ball_tree', weights='distance')
            self.classifier.fit(encodings, names)

        rospy.loginfo('Initialized')

    def recognize_person(self, image, distance_threshold=0.6):

        face_locations = face_recognition.face_locations(image, model='cnn')
        face_encodings = face_recognition.face_encodings(
            image, known_face_locations=face_locations, model='large')
        if self.fitting == 'svm':
            names = self.classifier.predict(face_encodings)
            return names, face_locations
        elif self.fitting == 'knn':
            closest_distances = self.classifier.kneighbors(
                face_encodings, n_neighbors=1)
            are_matches = [closest_distances[0][i][0] <=
                           distance_threshold for i in range(len(face_locations))]
            return [(pred, loc) if rec else ("unknown", loc)
                    for pred, loc, rec
                    in zip(self.classifier.predict(face_encodings), face_locations, are_matches)]
