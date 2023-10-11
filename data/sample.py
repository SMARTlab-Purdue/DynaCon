
def Your_role():

    message=[
        {"role": "user", "content": "You need to assist mobile robot which needs to go to the desired goal.\n"},
        {"role": "user", "content": "Find the desired object that is most related to the desired goal.\n"},
        {"role": "user", "content": "The mobile robot delivers you a list of objects detected while driving. The form is as follows: \n Object list =\n[object_name_A]\n[object_name_B]\n"},
        {"role": "user", "content": "The object list is delivered to you whenever the object list changes.\n"},
        {"role": "user", "content": "If there is an object in the object list that is most related to the desired goal, that object becomes output.\n"},
        {"role": "user", "content": "You need to find specific desired object using the context between each objects.\n"},
        {"role": "user", "content": "When you export an desired object as output, the mobile robot drives to that desired object and object list will be updated.\n"},
        {"role": "user", "content": "The form of the answer must be the following: 'Desired_object_name'.\n"},
        {"role": "user", "content": "You should not answer the sentence form.\n"},
        {"role": "user", "content": "Examples of scenarios are as follows:\n"}]


def Exameple01():

    message=[
        {"role": "user", "content": "Mobile robot's desired goal is Room 106\nObject list =\n[Room number plate 101]\n[Room number plate 110]\n[Room number plate 102]\n[Room number plate 109]\nDesired object: "},
        # Room 106 is very likely to be near the Room number plate 106.
        # You need to check the Room number plate's number.
        # Calculate the absolute difference between the desired goal and each room number plate. Select the one with the smallest absolute difference as the desired object.
        # In case of Room number plate 101, |106 - 101| = 5
        # In case of Room number plate 110, |106 - 110| = 4
        # In case of Room number plate 102, |106 - 102| = 4
        # In case of Room number plate 109, |106 - 109| = 3
        # The least absolute difference is Room number plate 109.
        {"role": "assistant", "content": "Room_number_plate_109"},
        {"role": "user", "content": "Mobile robot's desired goal is Room 106\nObject list =\n[Room number plate 102]\n[Room number plate 109]\n[Room number plate 103]\n[Room number plate 101]\n[Room number plate 110]\n[Room number plate 108]\nDesired object: "},
        # You need to check the Room number plate's number.
        # Calculate the absolute difference between the desired goal and each room number plate. Select the one with the smallest absolute difference as the desired object.
        # In case of Room number plate 102, |106 - 102| = 4
        # In case of Room number plate 109, |106 - 109| = 3
        # In case of Room number plate 103, |106 - 103| = 3
        # In case of Room number plate 101, |106 - 101| = 5
        # In case of Room number plate 110, |106 - 110| = 4
        # In case of Room number plate 108, |106 - 108| = 2
        # The least absolute difference is Room number plate 108.
        {"role": "assistant", "content": "Room_number_plate_108"},
        {"role": "user", "content": "Mobile robot's desired goal is Room 106\nObject list =\n[Room number plate 108]\n[Room number plate 103]\n[Room number plate 107]\n[Room number plate 109]\n[Room number plate 104]\n[Room number plate 102]\nDesired object: "},
        # In this case, Room number plate 107 is numerically the most close to the Room number plate 106.
        {"role": "assistant", "content": "Room_number_plate_107"},
        {"role": "user", "content": "Mobile robot's desired goal is Room 106\nObject list =\n[Room number plate 107]\n[Room number plate 104]\n[Room number plate 106]\n[Room number plate 108]\n[Room number plate 105]\n[Room number plate 103]\nDesired object: "},
        # In the Object list, there are Room_number_plate_106 and Room 106 is very likely to be near the Room number plate 106.
        {"role": "assistant", "content": "Room_number_plate_106"}]

def Exameple02():

    message=[
        {"role": "user", "content": "Mobile robot's desired goal is Room 202\nObject list =\n[Room number plate 212]\n[Room number plate 211]\n[Room number plate 206]\n[Room number plate 205]\nDesired object: "},
        # You need to check the Room number plate's number.
        # Calculate the absolute difference between the desired goal and each room number plate. Select the one with the smallest absolute difference as the desired object.
        # In case of Room number plate 212, |202 - 212| = 10
        # In case of Room number plate 211, |202 - 211| = 9
        # In case of Room number plate 206, |202 - 206| = 4
        # In case of Room number plate 205, |202 - 205| = 3
        # The least absolute difference is Room number plate 205.
        {"role": "assistant", "content": "Room_number_plate_205"},
        {"role": "user", "content": "Mobile robot's desired goal is Room 202\nObject list =\n[Room number plate 206]\n[Room number plate 204]\n[Room number plate 208]\n[Room number plate 205]\n[Room number plate 203]\n[Room number plate 207]\nDesired object: "},
        # Calculate the absolute difference between the desired goal and each room number plate. Select the one with the smallest absolute difference as the desired object.
        # In case of Room number plate 206, |202 - 206| = 4
        # In case of Room number plate 204, |202 - 204| = 2
        # In case of Room number plate 208, |202 - 208| = 6
        # In case of Room number plate 205, |202 - 205| = 3
        # In case of Room number plate 203, |202 - 203| = 1
        # In case of Room number plate 207, |202 - 207| = 5
        # The least absolute difference is Room number plate 203.
        {"role": "assistant", "content": "Room_number_plate_203"},
        {"role": "user", "content": "Mobile robot's desired goal is Room 202\nObject list =\n[Room number plate 204]\n[Room number plate 202]\n[Room number plate 206]\n[Room number plate 203]\n[Room number plate 201]\n[Room number plate 205]\nDesired object: "},
        {"role": "assistant", "content": "Room_number_plate_202"}]

def Exameple03():

    message=[
        {"role": "user", "content": "Mobile robot's desired goal is Living Room\nObject list =\n[painting1]\n[painting2]\n[painting3]\n[refrigerator]\n[microwave]\n[window1]\n[window2]\nDesired object: "},
        # The object "window" is more directly associated with a living room's context, as living rooms in many homes and apartments are typically designed with one or more windows to provide natural light and views. 
        # Thus, considering the context of a "Living Room", the "window" stands out as the most relevant object in this list.
        # You set the window1 as desired object.
        {"role": "assistant", "content": "window1"},
        {"role": "user", "content": "Mobile robot's desired goal is Living Room\nObject list =\n[painting1]\n[painting2]\n[painting3]\n[refrigerator]\n[microwave]\n[window1]\n[window2]\n[TV]\n[sofa]\nDesired object: "},
 
        {"role": "assistant", "content": "TV"}]