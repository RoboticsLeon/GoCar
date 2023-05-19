# !/usr/bin/env python3

from lxml import etree

# Load the XML file
tree = etree.parse('model.dae')
etree.strip_elements(tree,'{http://www.collada.org/2005/11/COLLADASchema}lines')

# Save the modified XML file
tree.write('model.dae')