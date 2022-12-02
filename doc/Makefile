
all: mpeg-pcc-tm2-sw-manual.pdf

clean:
	rm -rf *.log mpeg-pcc-tm2-sw-manual.pdf *.toc *.out *.blg $(patsubst %.md,%.tex,$(MD_SRC))

MD_SRC=\
	README.about.md \
	README.build.md \
	README.options.md \
	README.libraries.md \
	README.usage.md \
	README.references.md \
	$(NULL)

TEX_SRC=\
	mpeg-pcc-tm2-sw-manual.tex \
	$(patsubst %.md,%.tex,$(MD_SRC)) \
	$(NULL)

mpeg-pcc-tm2-sw-manual.pdf: $(TEX_SRC)

LATEX:=$(shell which xelatex || which pdflatex || which latex)

# for bibtex
%.aux: %.tex
	$(LATEX) $(LATEXFLAGS) $<

# normally this has a rule to use bibtool to pull in mpeg bib entries
%.bib: %.tex
	touch $@

%.bbl: %.tex %.aux %.bib
	-bibtex $(*F)

ifneq ($(notdir $(LATEX)),latex)
# for direct pdf generation
%.pdf: %.tex %.bbl
	$(LATEX) $(LATEXFLAGS) $<
	$(LATEX) $(LATEXFLAGS) $<
endif

%.pdf: %.dvi
	dvipdfm $*.dvi

%.dvi: %.tex %.bbl
	$(LATEX) $(LATEXFLAGS) $<
	$(LATEX) $(LATEXFLAGS) $<

# convert markdown text to latex for document generation
%.tex: %.md
	@echo pandoc -f markdown -t latex -o $@ $<
	pandoc -f markdown -t latex -o $@ $<
