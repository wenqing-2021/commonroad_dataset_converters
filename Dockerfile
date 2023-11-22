FROM python:3.9 AS build

RUN pip install poetry
WORKDIR /app

COPY commonroad_dataset_converter /app/commonroad_dataset_converter
COPY pyproject.toml /app/
COPY README.md /app/

RUN poetry export -o requirements.txt --without-hashes
RUN poetry build -f wheel

FROM python:3.9-slim AS app

RUN apt-get update && apt-get install -y gcc cmake

COPY --from=build /app/requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir --upgrade -r /app/requirements.txt && rm /app/requirements.txt

ARG WHEEL_NAME="commonroad_dataset_converter-2023.2-py3-none-any.whl"
COPY --from=build /app/dist/${WHEEL_NAME} /app/
RUN pip install /app/${WHEEL_NAME} && rm /app/${WHEEL_NAME}

ENTRYPOINT ["crconvert"]
CMD ["--help"]