
import cv2
import numpy as np

def insideOtherRect(rect, otherRects, wMin = 20, hMin = 20):
  x1, y1, w1, h1 = rect
  for i in range(len(otherRects)):
    x2, y2, w2, h2 = cv2.boundingRect(otherRects[i])
    if x1 > x2 and x1 < x2 + w2 and y1 > y2 and y1 < y2 + h2:
      return True
  return False

# This takes a greyscale input, img, and finds all instances of high color values above threshHold
def findColorRaw(img, threshHold):

  # Blur image
  blur = cv2.bilateralFilter(img,9,75,75)
  blur = cv2.GaussianBlur(blur,(5,5),0)

  _, thresh = cv2.threshold(blur,threshHold,255,cv2.THRESH_BINARY)#140, 255
  opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
  _, contours, h = cv2.findContours(opening,1,2)

  rects = []
  for j in range(len(contours)):
    x, y, w, h = cv2.boundingRect(contours[j])
    if w > wMin and h > hMin and not insideOtherRect((x, y, w, h), contours):
      rects.append((x, y, w, h))

  return rects

# wrapper for findColorRaw. This takes red, green or blue and finds it.
# NOTE: This is tuned for an old laptop webcam. PLEASE change the threshHold values if needed.
def findColor(img, colorName):
  lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
  l, a, b = cv2.split(lab)

  if colorName == "red":
    return findColorRaw(a, 165)
  elif colorName == "green":
    return findColorRaw(cv2.bitwise_not(a), 150)
  elif colorName == "yellow": # Not very reliable; don't use
    return findColorRaw(b, 150)
  elif colorName == "blue":
    return findColorRaw(cv2.bitwise_not(b), 160)
  else:
    return []

cap = cv2.VideoCapture(0)
kernel = np.ones((5, 5), np.uint8)
wMin = 20
hMin = 20

while True:
  ret, img = cap.read()

  if(ret == False):
    print("CAMERA DIED IN FIRE")
    break

  if cv2.waitKey(1) & 0xFF == ord("q"):
    break

  for rect in findColor(img, "red"):
    x, y, w, h = rect
    cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)

  for rect in findColor(img, "blue"):
    x, y, w, h = rect
    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

  for rect in findColor(img, "green"):
    x, y, w, h = rect
    cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

  cv2.imshow("IMAGE", img)

cap.release()
cv2.destroyAllWindows()
