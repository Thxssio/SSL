VENV_DIR := .venv
VENV_BIN := $(VENV_DIR)/bin
PYTHON := $(VENV_BIN)/python
PIP := $(VENV_BIN)/pip

.PHONY: setup install clean shell

setup:
	python3 -m venv $(VENV_DIR)
	$(PYTHON) -m pip install --upgrade pip
	$(PYTHON) -m pip install -r requirements.txt

install: setup

shell:
	. $(VENV_BIN)/activate; exec bash -i

clean:
	rm -rf $(VENV_DIR)
