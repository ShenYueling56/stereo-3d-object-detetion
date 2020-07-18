from PIL import Image
import os.path
import glob

def convertjpg(jpgfile,outdir,width=640,height=480):
  img=Image.open(jpgfile)
  try:
    new_img=img.resize((width,height),Image.BILINEAR)  
    new_img.save(os.path.join(outdir,os.path.basename(jpgfile)))
  except Exception as e:
    print(e)


for jpgfile in glob.glob("/home/shenyl/Documents/sweeper/data/0716/background/*.jpg"):
  convertjpg(jpgfile, "/home/shenyl/Documents/sweeper/data/0716/bk_croped")

