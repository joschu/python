from __future__ import division


def create_box_from_bounds(env, bounds, name="box"):
  xmin, xmax, ymin, ymax, zmin, zmax = bounds
  xml_str = """
<Environment>
  <KinBody name="%s">
    <Body type="static">
      <Geom type="box">
	<Translation> %f %f %f </Translation>
	<extents> %f %f %f </extents>
      </Geom>
    </Body>
  </KinBody>

</Environment>
"""%( name,
      (xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2,
      (xmax-xmin)/2, (ymax-ymin)/2, (zmax-zmin)/2 )
  
  
  fname = "/tmp/%s.xml"%name
  with open(fname,"w") as fh:
    fh.write(xml_str)
      
  return env.Load(fname)

def create_cylinder(env, center, radius, height, name = "cylinder"):
  xcenter, ycenter, zcenter = center
  xml_str = """
<Environment>
  <KinBody name="%s">
    <Body type="static">
      <Geom type="cylinder">
	<Translation> %f %f %f </Translation>
        <RotationAxis>1 0 0 90</RotationAxis>
	<radius> %f </radius>
        <height> %f </height>
      </Geom>
    </Body>
  </KinBody>

</Environment>
"""%( name,
      xcenter, ycenter, zcenter,
      radius, height )
  
  fname = "/tmp/%s.xml"%name
  with open(fname,"w") as fh:
    fh.write(xml_str)
      
  return env.Load(fname)

if __name__ == "__main__":
  import openravepy
  env = openravepy.Environment()
  print create_cylinder(env, (0,0,0), 1, 2)
  print create_box_from_bounds(env, (0,1,0,1,0,1))
  env.SetViewer("qtcoin")
  
                        