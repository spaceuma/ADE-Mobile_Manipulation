#include <proxy_library/visualization/GridNode.hpp>

#include <osg/Point>
#include <osgText/Text>
#include <boost/lexical_cast.hpp>

namespace proxy_library 
{
    
GridNode::GridNode(int rows,int cols,float dx, float dy, bool show_coordinates, 
                   const ::osg::Vec4 &color)
{
    float size_x = cols*dx;
    float size_y = rows*dy;
    float size = std::min(size_x,size_y);
    float interval = std::min(dx,dy);

    osg::ref_ptr<osg::Geode> geode = new ::osg::Geode;
    this->addChild(geode);

    osg::ref_ptr<osg::Geometry> geom = new ::osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> v = new ::osg::Vec3Array;
    geom->setVertexArray(v);

    // Draw grid lines
    for(float x = - size_x*0.5f; x <= size_x*0.5f; x += dx)
    {
        v->push_back( ::osg::Vec3(x, -size_y*0.5f, 0.01f));
        v->push_back( ::osg::Vec3(x, size_y*0.5f, 0.01f));
    }
    for(float y = - size_y*0.5f; y <= size_y*0.5f; y += dy)
    {
        v->push_back( ::osg::Vec3(-size_x*0.5f, y, 0.01f));
        v->push_back( ::osg::Vec3(size_x*0.5f, y, 0.01f));
    }

    // Draw concentric circles
    for(float r=0; r<size*0.5f; r+=interval)
    {
        float xp = (2.0*M_PI)/(r*100);
        for(float x=0; x<2.0*M_PI; x+=2*xp)
        {
            v->push_back( ::osg::Vec3(cos(x)*r, sin(x)*r, 0.01f) );
            v->push_back( ::osg::Vec3(cos(x+xp)*r, sin(x+xp)*r, 0.01f) );
        }
    }

    // Draw coordinates
    if(show_coordinates)
    {
        for(float x = - size_x*0.5f; x <= size_x*0.5f; x += dx)
        {
            for(float y = - size_y*0.5f; y <= size_y*0.5f; y += dy)
            {
                osg::ref_ptr<osg::Geode> text_geode = new osg::Geode;
                osg::ref_ptr<osgText::Text> text= new osgText::Text;
                std::string label = "(";
                label += boost::lexical_cast<std::string>(x);
                label += ",";
                label += boost::lexical_cast<std::string>(y);
                label += ")";
                text->setText(label);
                text->setCharacterSize(interval * 0.1);
                text->setPosition(osg::Vec3d(x+0.02, y+0.05, 0.01f));
                text_geode->addDrawable(text);
                this->addChild(text_geode);
            }
        }
    }

    // Set colors
    osg::ref_ptr<osg::Vec4Array> c = new ::osg::Vec4Array;
    geom->setColorArray(c);
    geom->setColorBinding( ::osg::Geometry::BIND_OVERALL );
    c->push_back(color);

    // Draw a four-vertex quad from the stored data.
    geom->addPrimitiveSet(new ::osg::DrawArrays(::osg::PrimitiveSet::LINES,0,v->size()));

    osg::ref_ptr<osg::StateSet> stategeode = geode->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, ::osg::StateAttribute::OFF );
    geode->addDrawable(geom);
}

} // end namespace proxy_library
