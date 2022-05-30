%%
%% mpegdoc: Copyright (c) 2011 BBC Research & Development
%% All rights reserved.
%%
%% Permission to use, copy, modify, and/or distribute this software for any
%% purpose with or without fee is hereby granted, provided that the above
%% copyright notice and this permission notice appear in all copies.
%%
%% Neither the names of the BBC, ITU, ISO, IEC nor the names of its
%% contributors may be used to endorse or promote products derived from
%% this software without specific prior written permission.
%%
%% THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
%% WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
%% MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
%% ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
%% WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
%% ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
%% OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
%%
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{mpegdoc}[2011/06/01 MPEG document class]

\LoadClassWithOptions{article}

% typeset using Times New Roman
\RequirePackage{ifxetex}
\ifxetex
  % use the correct fonts when using XeTeX
  \RequirePackage{fontspec}
  \defaultfontfeatures{Mapping=tex-text}
  \setmainfont{Times New Roman}
\else
  \RequirePackage{times}
\fi

% expand the margins
\RequirePackage{geometry}
\geometry{tmargin=1.6cm,lmargin=1in,rmargin=2.5cm,bmargin=1in,nohead}

% require graphicx for loading the logos in the header
\RequirePackage{graphicx}

% don't number the abstract
\renewenvironment{abstract}{\section*{Abstract}}{}

% format the date in iso style
\RequirePackage{datetime}
\renewcommand{\dateseparator}{-}
\newdateformat{MPEG@yyyymmdddate}{%
  \THEYEAR\dateseparator\twodigit{\THEMONTH}\dateseparator\twodigit{\THEDAY}}
 % 2020-06-06}


% all pages only have a footer
\RequirePackage{fancyhdr}
\pagestyle{fancy}
\renewcommand{\headrulewidth}{0pt}
\fancyhead{}
\cfoot{\thepage}
\rfoot{\MPEG@yyyymmdddate\today}

% macros for document metadata
\RequirePackage[normalem]{ulem}
\def\@mpegdocnum{\uline{xxx}}
\def\@mpegdocstatus{\uline{TODO: Add \texttt{\textbackslash{}mpegdocstatus}}}
\def\@mpegdocpurpose{\uline{TODO: Add \texttt{\textbackslash{}mpegdocpurpose}}}
\def\@mpegdocsource{\uline{TODO: Add \texttt{\textbackslash{}mpegdocsource}}}
\def\@mpegmeeting{\uline{TODO: Add \texttt{\textbackslash{}mpegmeeting}}}
\newcommand{\mpegdocnum}[1]{\def\@mpegdocnum{\mbox{#1}}}
\newcommand{\mpegdocstatus}[1]{\def\@mpegdocstatus{\mbox{#1}}}
\newcommand{\mpegdocpurpose}[1]{\def\@mpegdocpurpose{\mbox{#1}}}
\newcommand{\mpegdocsource}[1]{\def\@mpegdocsource{\mbox{#1}}}
\newcommand{\mpegmeeting}[1]{\def\@mpegmeeting{\mbox{#1}}}

% affiliation block in the author list
\newenvironment{affiliation}{%
	\begin{tabular}[t]{@{}>{\renewcommand{\\}{, }}l@{}}%
}{%
	\end{tabular}%
}

\RequirePackage{array}
  
\RequirePackage{ifthen}
\newcounter{mpeg@author@column}
\newcommand{\@mpeg@switch@author@column}{%
	\ifthenelse{\value{mpeg@author@column} = 1}{%
		% do nothing
	}{%
		\addtocounter{mpeg@author@column}{1}%
		&
	}%
}

\newcommand*{\email}[1]{\@mpeg@switch@author@column\textit{#1}}
\newcommand*{\tel}[1]{\@mpeg@switch@author@column\textrm{#1}}

% Make a title at the top of the current page.
\renewcommand\maketitle{%
	% none of the title is put in the header (it is too difficult to
	% make the page match the word style in this case)
	\vspace*{-1.7cm}
	\begin{raggedright}
	\rule{0pt}{0.74cm}% strut incase images aren't loaded
	\IfFileExists{logos/itu}{\includegraphics[height=0.74cm]{logos/itu}}{}
	\IfFileExists{logos/iso}{\includegraphics[height=0.74cm]{logos/iso}}{}
	\IfFileExists{logos/iec}{\includegraphics[height=0.74cm]{logos/iec}}{}
	\\
	\textbf{Moving Picture Experts Group (MPEG)}\\[0ex]
	\textbf{of ISO/IEC JTC 1/SC 29/WG 7}
		\hfill Document: MPEG-\@mpegdocnum\\[0ex]
	\@mpegmeeting

	\vspace{0.5\baselineskip}

	\newcommand{\@strutb}{\rule{0pt}{2.5ex}}
	\begin{tabular}{lp{0.78\textwidth}}
	\@strutb \it Title: & \@title \\
	\@strutb \it Status: & \@mpegdocstatus \\
	\@strutb \it Purpose: & \@mpegdocpurpose \\
	\@strutb \it Author(s): & %
		\setcounter{mpeg@author@column}{0}
		\let\@and\\
		\renewcommand{\and}{\@and\setcounter{mpeg@author@column}{0}}
		\newcommand{\@NLtoPAR}{\renewcommand{\\}{\par}}
		\begin{tabular}[t]{@{}>{\@NLtoPAR}p{3in}|>{\@NLtoPAR}p{2in}@{}}%
			\@author
		\end{tabular}\\
	\@strutb \it Source: & \@mpegdocsource \\
	\end{tabular}
	\end{raggedright}

	% draw a short horizontal line to delimit the title from body
	{\center\rule{0.35\textwidth}{1pt}\\}
}

%
\RequirePackage{parskip}
\RequirePackage[compact]{titlesec}
\titlespacing{\section}{0pt}{*0}{*0}
\titlespacing{\subsection}{0pt}{*0}{*0}
\titlespacing{\subsubsection}{0pt}{*0}{*0}
