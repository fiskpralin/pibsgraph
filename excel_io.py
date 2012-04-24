try:
	import xlwt
	import xlrd
	import xlutils
	from xlutils.copy import copy as xlcpy
	from xlutils.styles import Styles
	from xlutils.filter import process,XLRDReader,XLWTWriter
except:
	raise Exception('the xlwt, xlutils and xlrd package are needed to write excel files. Please download and install it.')


# Patch: add this function to the end of xlutils/copy.py
def copy2(wb):
    w = XLWTWriter()
    process(
        XLRDReader(wb,'unknown.xls'),
        w
        )
    return w.output[0][1], w.style_list

class ExcelOutput():
	"""
	When an excel file is read, copied and modified, an object of this class is called.
	>>> e=ExcelOutput()
	>>> e.modify(row,col,content)
	>>> e.save()

	"""
	def __init__(self, template='sim/planting/template.xls', out='test.xls' ):
		self.rdbook = xlrd.open_workbook(template, formatting_info=True)
		sheetx = 0 #default, try first sheet
		self.wtbook, self.style_list = copy2(self.rdbook)
		self.changeSheet(sheetx)
		self.rowMax=None
		self.output=out
	def changeSheet(self, sheetx):
		self.rdsheet = self.rdbook.sheet_by_index(sheetx)
		self.wtsheet = self.wtbook.get_sheet(sheetx)
	def save(self):
		self.wtbook.save(self.output)
	def modify(self,row,col,content):
		"""
		modify cell (row, col) and set its content
		"""
		if self.rowMax: #skip the style
			self.wtsheet.write(row, col, content)
		else:
			try:
				xf_index = self.rdsheet.cell_xf_index(row, col)
				self.wtsheet.write(row, col, content, self.style_list[xf_index])
			except:
				self.rowMax=row-1
				self.wtsheet.write(row, col, content)
				
		




