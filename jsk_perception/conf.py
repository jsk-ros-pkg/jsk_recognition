
# -*- coding: utf-8 -*-
import sys, os

extensions = ['sphinx.ext.autodoc', 'sphinx.ext.intersphinx', 'sphinx.ext.todo', 'sphinx.ext.ifconfig']
templates_path = ['.templates']
source_suffix = '.rst'
#source_encoding = 'utf-8'
master_doc = 'index'
project = u'jsk_perception'
copyright = u''

version = '1.0'
release = '1.0'
#language = None
#today_fmt = '%B %d, %Y'
exclude_patterns = []
#default_role = None
#add_function_parentheses = True
#add_module_names = True
show_authors = True
pygments_style = 'sphinx'
#modindex_common_prefix = []

# -- Options for HTML output ---------------------------------------------------
html_theme = 'sphinxdoc'
#html_theme_options = {}
#html_theme_path = []
#html_title = None
#html_short_title = None
#html_logo = None
#html_favicon = None
html_static_path = ['.static']
#html_last_updated_fmt = '%b %d, %Y'
#html_use_smartypants = True
#html_sidebars = {}
#html_additional_pages = {}
#html_domain_indices = True
#html_use_index = True
#html_split_index = False
#html_show_sourcelink = True
#html_show_sphinx = True
#html_show_copyright = True
#html_use_opensearch = ''
#html_file_suffix = None
htmlhelp_basename = 'jsk_perception'

intersphinx_mapping = {'http://docs.python.org/': None}
